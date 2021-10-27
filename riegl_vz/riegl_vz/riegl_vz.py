import sys
import os
import time
import json
import subprocess
import threading
import numpy as np
from datetime import datetime
from os.path import join, dirname, abspath

from std_msgs.msg import (
    Header
)
from sensor_msgs.msg import (
    PointCloud2,
    PointField
)
from geometry_msgs.msg import (
    PoseWithCovariance
)
from nav_msgs.msg import (
    Path,
    Odometry
)
import std_msgs.msg as std_msgs
import builtin_interfaces.msg as builtin_msgs

from rclpy.node import Node

import riegl.rdb

from vzi_services.controlservice import ControlService
from vzi_services.projectservice import ProjectService
from vzi_services.dataprocservice import DataprocService
from vzi_services.scannerservice import ScannerService

from .pose import (
    readVop,
    readAllSopv
)
from .ssh import RemoteClient
from .utils import (
    SubProcess
)

appDir = dirname(abspath(__file__))

class ScanPattern(object):
    def __init__(self):
        self.lineStart = 30.0
        self.lineStop = 130.0
        self.lineIncrement = 0.04
        self.frameStart = 0.0
        self.frameStop = 360.0
        self.frameIncrement = 0.04
        self.measProgram = 3

class Status(object):
    def __init__(self):
        self.opstate = "unavailable"
        self.progress = 0

class StatusMaintainer(object):
    def __init__(self):
        self._status: Status = Status()
        self._threadLock = threading.Lock()

    def _lock(self):
        self._threadLock.acquire()

    def _unlock(self):
        self._threadLock.release()

    def setOpstate(self, opstate):
        self._lock()
        self._status.opstate = opstate
        self._unlock()

    def setProgress(self, progress):
        self._lock()
        #if self._status.opstate == "scanning":
        self._status.progress = progress
        self._unlock()

    def getStatus(self):
        status: Status
        self._lock()
        status = self._status
        self._unlock()
        return status

class RieglVz():
    def __init__(self, node):
        self.hostname = node.hostname
        self.sshUser = node.sshUser
        self.sshPwd = node.sshPwd
        self.workingDir = node.workingDir
        self._node = node
        self._logger = node.get_logger()
        self._connectionString = self.hostname + ":20000"
        self.scanposName = None
        self._status: StatusMaintainer = StatusMaintainer()
        self._path = Path()
        self._stopReq = False
        self._shutdownReq = False
        self._ctrlSvc = None

        self.scanPublishFilter = node.scanPublishFilter
        self.scanPublishLOD = node.scanPublishLOD

        if not os.path.exists(self.workingDir):
            os.mkdir(self.workingDir)

        self._statusThread = threading.Thread(target=self._statusThreadFunc, args=())
        self._statusThread.daemon = True
        self._statusThread.start()

    def _statusThreadFunc(self):
        def onTaskProgress(arg0):
            obj = json.loads(arg0)
            self._logger.debug("scan progress: {0} % ({1}, {2})".format(obj["progress"], obj["id"], obj["progresstext"]))
            if obj["id"] == 1:
                self._status.setProgress(obj["progress"])
            self._node._statusUpdater.force_update()

        while self._ctrlSvc is None:
            try:
                self._ctrlSvc = ControlService(self._connectionString)
                self.taskProgress = self._ctrlSvc.taskProgress().connect(onTaskProgress)
                self._status.setOpstate("waiting")
            except:
                self._logger.debug("Scanner is not available!")
            time.sleep(1.0)

    def resetPath(self):
        self._path = Path()

    def _downloadFile(self, remoteFile: str, localFile: str):
        self._logger.debug("Downloading file..")
        self._logger.debug("remote file = {}".format(remoteFile))
        self._logger.debug("local file  = {}".format(localFile))
        ssh = RemoteClient(host=self.hostname, user=self.sshUser, password=self.sshPwd)
        ssh.downloadFile(filepath=remoteFile, localpath=localFile)
        ssh.disconnect()
        self._logger.debug("File download finished")
        return localFile

    def _getProjectPath(self):
        projSvc = ProjectService(self._connectionString)
        return projSvc.projectPath()

    def _getScanposPath(self, scanposName: str):
        return self._getProjectPath() + '/' + str(scanposName) + '.SCNPOS/scans'

    def _getScanId(self, scanposName: str):
        if int(scanposName) == 0:
            procSvc = DataprocService(self._connectionString)
            return procSvc.actualFile(0)

        ssh = RemoteClient(host=self.hostname, user=self.sshUser, password=self.sshPwd)
        scanposPath = self._getScanposPath(scanposName)
        cmd = ["ls -t", scanposPath + "/*.rxp"]
        self._logger.debug("CMD = {}".format(" ".join(cmd)))
        response = ssh.executeCommand(" ".join(cmd))
        ssh.disconnect()

        if len(response) == 0:
            return "null"

        return (scanposPath + "/" + os.path.basename(response[0]).split(".")[0] + ".rxp").replace("/media/", "")

    def _getTimeStampFromScanId(self, scanId: str):
        scanFileName: str = os.path.basename(scanId)
        dateTime = datetime.strptime(scanFileName, '%y%m%d_%H%M%S.rxp')
        #self._logger.debug("dateTime = {}".format(dateTime))
        return int(dateTime.strftime("%s"))

    def getPointCloud(self, scanposName: str, pointcloud: PointCloud2):
        if self.getStatus().opstate == "unavailable":
            return False, pointcloud

        scanId = self._getScanId(scanposName)
        self._logger.debug("scan id = {}".format(scanId))

        if scanId == "null":
            return False, pointcloud

        remoteFile = "/media/" + scanId.replace(".rxp", ".rdbx")
        localFile = self.workingDir + "/scan.rdbx"
        self._downloadFile(remoteFile, localFile)

        self._logger.debug("Generate point cloud..")
        with riegl.rdb.rdb_open(localFile) as rdb:
            ts = builtin_msgs.Time(sec = self._getTimeStampFromScanId(scanId), nanosec = 0)
            filter = ""
            rosDtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize

            numTotalPoints = 0
            numPoints = 0
            data = bytearray()
            scanPublishLOD = self.scanPublishLOD
            if self.scanPublishLOD < 0:
                scanPublishLOD = 0
            for points in rdb.select(
                self.scanPublishFilter,
                chunk_size=100000
                ):
                pointStep = 2 ** scanPublishLOD
                for point in points:
                    if not (numTotalPoints % pointStep):
                        data.extend(point["riegl.xyz"].astype(dtype).tobytes())
                        data.extend(point["riegl.reflectance"].astype(dtype).tobytes())
                        numPoints += 1
                    numTotalPoints += 1

            fields = [PointField(
                name = n, offset = i*itemsize, datatype = rosDtype, count = 1)
                for i, n in enumerate('xyzr')]

            header = std_msgs.Header(frame_id = "riegl_vz_socs", stamp = ts)

            pointcloud = PointCloud2(
                header = header,
                height = 1,
                width = numPoints,
                is_dense = False,
                is_bigendian = False,
                fields = fields,
                point_step = (itemsize * 4),
                row_step = (itemsize * 4 * numPoints),
                data = data
            )
            #for point in rdb.points():
            #    self._logger.debug("{0}".format(point.riegl_xyz))
        self._logger.debug("Point cloud generated")

        return True, pointcloud

    def _scanThreadFunc(self):
        self._status.setOpstate("scanning")
        self._status.setProgress(0)

        self._logger.info("Starting data acquisition..")
        self._logger.info("project name = {}".format(self.projectName))
        self._logger.info("scanpos name = {}".format(self.scanposName))
        self._logger.info("storage media = {}".format(self.storageMedia))
        self._logger.info("scan pattern = {0}, {1}, {2}, {3}, {4}, {5}".format(
            self.scanPattern.lineStart,
            self.scanPattern.lineStop,
            self.scanPattern.lineIncrement,
            self.scanPattern.frameStart,
            self.scanPattern.frameStop,
            self.scanPattern.frameIncrement))
        self._logger.info("meas program = {}".format(self.scanPattern.measProgram))
        self._logger.info("scan publish = {}".format(self.scanPublish))
        self._logger.info("scan publish filter = '{}'".format(self.scanPublishFilter))
        self._logger.info("scan publish LOD = {}".format(self.scanPublishLOD))
        self._logger.info("scan register = {}".format(self.scanRegister))

        scriptPath = join(appDir, "acquire-data.py")
        cmd = [
            "python3", scriptPath,
            "--connectionstring", self._connectionString,
            "--project", self.projectName,
            "--scanposition", self.scanposName,
            "--storage-media", str(self.storageMedia)]
        if self.reflSearchSettings:
            rssFilePath = join(self.workingDir, "reflsearchsettings.json")
            with open(rssFilePath, "w") as f:
                json.dump(self.reflSearchSettings, f)
            cmd.append("--reflsearch")
            cmd.append(rssFilePath)
        if self.scanPattern:
            cmd.extend([
                "--line-start", str(self.scanPattern.lineStart),
                "--line-stop", str(self.scanPattern.lineStop),
                "--line-incr", str(self.scanPattern.lineIncrement),
                "--frame-start", str(self.scanPattern.frameStart),
                "--frame-stop", str(self.scanPattern.frameStop),
                "--frame-incr", str(self.scanPattern.frameIncrement),
                "--measprog", str(self.scanPattern.measProgram)
            ])
        if self.captureImages:
            cmd.extend([
                "--capture-images",
                "--capture-mode", str(self.captureMode),
                "--image-overlap", str(self.imageOverlap)
            ])
        self._logger.debug("CMD = {}".format(" ".join(cmd)))
        subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self._logger.debug("Subprocess started.")
        subproc.waitFor(errorMessage="Data acquisition failed.", block=True)
        if self._stopReq:
            self._stopReq = False
            self._status.setOpstate("waiting")
            self._logger.info("Scan stopped")
            return
        self._logger.info("Data acquisition finished")

        self._status.setOpstate("processing")

        self._logger.info("Converting RXP to RDBX..")
        scriptPath = join(appDir, "create-rdbx.py")
        cmd = [
            "python3", scriptPath,
            "--connectionstring", self._connectionString,
            "--project", self.projectName,
            "--scanposition", self.scanposName]
        self._logger.debug("CMD = {}".format(" ".join(cmd)))
        subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self._logger.debug("Subprocess started.")
        subproc.waitFor("RXP to RDBX conversion failed.")
        if self._stopReq:
            self._stopReq = False
            self._status.setOpstate("waiting")
            self._logger.info("Scan stopped")
            return
        self._logger.info("RXP to RDBX conversion finished")

        if self.scanPublish:
            self._logger.info("Downloading and publishing point cloud..")
            pointcloud: PointCloud2 = PointCloud2()
            ok, pointcloud = self.getPointCloud(self.scanposName, pointcloud)
            if ok:
                self._node.pointCloudPublisher.publish(pointcloud)
            self._logger.info("Point cloud published")

        if self.scanRegister:
            self._logger.info("Starting registration..")
            scriptPath = os.path.join(appDir, "register-scan.py")
            cmd = [
                "python3", scriptPath,
                "--connectionstring", self._connectionString,
                "--project", self.projectName,
                "--scanposition", self.scanposName]
            self._logger.debug("CMD = {}".format(" ".join(cmd)))
            subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
            subproc.waitFor(errorMessage="Registration failed.", block=True)
            if self._stopReq:
                self._stopReq = False
                self._status.setOpstate("waiting")
                self._logger.info("Scan stopped")
                return
            self._logger.info("Registration finished")

            self._logger.info("Downloading and publishing pose..")
            ok, sopv = self.getSopv()
            if ok:
                # publish pose
                self._node.posePublisher.publish(sopv.pose)
                # publish path
                self._path.header = sopv.pose.header
                self._path.poses.append(sopv.pose)
                self._node.pathPublisher.publish(self._path)
                #publish odometry
                odom = Odometry(
                    header = Header(frame_id = "riegl_vz_vocs"),
                    child_frame_id = "riegl_vz_socs",
                    pose = PoseWithCovariance(pose = sopv.pose.pose)
                )
                self._node.odomPublisher.publish(odom)
            self._logger.info("Pose published")

        self._status.setOpstate("waiting")

    def scan(
        self,
        projectName: str,
        scanposName: str,
        storageMedia: int,
        scanPattern: ScanPattern,
        scanPublish: bool = True,
        scanPublishFilter: str = "",
        scanPublishLOD: int = 1,
        scanRegister: bool = True,
        reflSearchSettings: dict = None,
        captureImages: bool = False,
        captureMode: int = 1,
        imageOverlap: int = 25):
        """Acquire data at scan position.

        Args:
          projectName ... the project name
          scanposName ... the name of the new scan position
          storageMedia ... storage media for data recording
          scanPattern ... the scan pattern"""

        if self.getStatus().opstate == "unavailable":
            return False

        if self.isBusy():
            return False

        self.projectName = projectName
        self.scanposName = scanposName
        self.storageMedia = storageMedia
        self.scanPattern = scanPattern
        self.scanPublish = scanPublish
        self.scanPublishFilter = scanPublishFilter
        self.scanPublishLOD = scanPublishLOD
        self.scanRegister = scanRegister
        self.reflSearchSettings = reflSearchSettings
        self.captureImages = captureImages
        self.captureMode = captureMode
        self.imageOverlap = imageOverlap

        thread = threading.Thread(target=self._scanThreadFunc, args=())
        thread.daemon = True
        thread.start()

        while not self.isScanning(block=False):
            time.sleep(0.2)

        return True

    def isScanning(self, block = True):
        if block:
            while self.getStatus().opstate == "scanning":
                time.sleep(0.2)
        return True if self.getStatus().opstate == "scanning" else False

    def isBusy(self, block = True):
        if block:
            while self.getStatus().opstate != "waiting":
                time.sleep(0.2)
        return False if self.getStatus().opstate == "waiting" else True

    def getStatus(self):
        return self._status.getStatus()

    def getAllSopv(self):
        if self.scanposName is None:
            return False, None

        if self.getStatus().opstate == "unavailable":
            return False, None

        sopvFileName = "all_sopv.csv"
        remoteFile = self._getProjectPath() + "/Voxels1.VPP/" + sopvFileName
        localFile = self.workingDir + "/" + sopvFileName
        self._downloadFile(remoteFile, localFile)

        ok = True
        sopvs = readAllSopv(localFile, self._logger)

        return ok, sopvs

    def getSopv(self):
        if self.scanposName is None:
            return False, None

        if self.getStatus().opstate == "unavailable":
            return False, None

        ok, sopvs = self.getAllSopv()
        if ok and len(sopvs):
            sopv = sopvs[-1]
        else:
            sopv = None
            ok = False

        return ok, sopv

    def getVop(self):
        if self.scanposName is None:
            return False, None

        if self.getStatus().opstate == "unavailable":
            return False, None

        sopvFileName = "VPP.vop"
        remoteFile = self._getProjectPath() + "/Voxels1.VPP/" + sopvFileName
        localFile = self.workingDir + "/" + sopvFileName
        self._downloadFile(remoteFile, localFile)

        vop = readVop(localFile)

        return True, vop

    def stop(self):
        self._stopReq = True

        if self.getStatus().opstate != "unavailable":
            ctrlSvc = ControlService(self._connectionString)
            ctrlSvc.stop()
            self.isBusy()

    def shutdown(self):
        self.stop()
        if self.getStatus().opstate != "unavailable":
            scnSvc = ScannerService(self._connectionString)
            scnSvc.shutdown()
