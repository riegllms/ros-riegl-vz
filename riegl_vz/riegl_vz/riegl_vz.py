import sys
import os
import time
import subprocess
import threading
import numpy as np
from datetime import datetime
from os.path import join, dirname, abspath

from sensor_msgs.msg import (
    PointCloud2,
    PointField
)
import std_msgs.msg as std_msgs
import builtin_interfaces.msg as builtin_msgs

from rclpy.node import Node

import riegl.rdb

from vzi_services.controlservice import ControlService
from vzi_services.projectservice import ProjectService
from vzi_services.dataprocservice import DataprocService

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
        self.opstate = "waiting"

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

        if not os.path.exists(self.workingDir):
            os.mkdir(self.workingDir)

    def _downloadFile(self, remoteFile: str, localFile: str):
        self._logger.info("Downloading file..")
        self._logger.debug("remote file = {}".format(remoteFile))
        self._logger.debug("local file  = {}".format(localFile))
        ssh = RemoteClient(host=self.hostname, user=self.sshUser, password=self.sshPwd)
        ssh.downloadFile(filepath=remoteFile, localpath=localFile)
        ssh.disconnect()
        self._logger.info("File download finished")
        return localFile

    def _getProjectPath(self):
        projSvc = ProjectService(self._connectionString)
        return projSvc.projectPath()

    def _getScanposPath(self, scanposName: str):
        return self._getProjectPath() + '/' + scanposName + '.SCNPOS/scans'

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
        return (scanposPath + "/" + os.path.basename(response[0]).split(".")[0] + ".rxp").replace("/media/", "")

    def _getTimeStampFromScanId(self, scanId: str):
        scanFileName: str = os.path.basename(scanId)
        dateTime = datetime.strptime(scanFileName, '%y%m%d_%H%M%S.rxp')
        #self._logger.debug("dateTime = {}".format(dateTime))
        return int(dateTime.strftime("%s"))

    def downloadAndPublishScan(self, scanposName: str, pointcloud: PointCloud2):
        scanId = self._getScanId(scanposName)
        self._logger.debug("scan id = {}".format(scanId))
        remoteFile = "/media/" + scanId.replace(".rxp", ".rdbx")
        localFile = self.workingDir + "/scan.rdbx"
        self._downloadFile(remoteFile, localFile)

        self._logger.info("Extracting and publishing point cloud..")
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

            header = std_msgs.Header(frame_id = "RIEGL_SOCS", stamp = ts)

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

            self._node.pointCloudPublisher.publish(pointcloud)
        self._logger.info("Point cloud published")

        return True, pointcloud

    def _scanThread(self):
        self._status.setOpstate("scanning")

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
        #while not subproc.waitFor(errorMessage="Data acquisition failed.", block=False):
        #    time.sleep(1.0)
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
        self._logger.info("RXP to RDBX conversion finished")

        if self.scanPublish:
            pointcloud: PointCloud2 = PointCloud2()
            ok, pointcloud = self.downloadAndPublishScan(self.scanposName, pointcloud)

        if self.scanRegister:
            self._logger.info("Starting registration..")
            scriptPath = os.path.join(appDir, "bin", "register-scan.py")
            cmd = [
                "python3", scriptPath,
                "--project", self.projectName,
                "--scanposition", self.scanposName]
            self._logger.debug("CMD = {}".format(" ".join(cmd)))
            subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
            subproc.waitFor(errorMessage="Registration failed.", block=True)
            #while not subproc.waitFor(errorMessage="Registration failed.", block=False):
            #    time.sleep(1.0)
            self._logger.info("Registration finished")

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

        thread = threading.Thread(target=self._scanThread, args=())
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

        sopvFileName = "all_sopv.csv"
        remoteFile = "/media/" + self._getProjectPath() + "/Voxels1.VPP/" + sopvFileName
        localFile = self.workingDir + "/" + sopvFileName
        self._downloadFile(remoteFile, localFile)

        ok = True
        sopvs = readAllSopv(localFile)

        return ok, sopvs

    def getSopv(self):
        if self.scanposName is None:
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

        vop: geometry_msgs.PoseStamped
        sopvFileName = "VPP.vop"
        remoteFile = "/media/" + self._getProjectPath() + "/Voxels1.VPP/" + sopvFileName
        localFile = self.workingDir + "/" + sopvFileName
        self._downloadFile(remoteFile, localFile)

        vop = readVop()

        return True, vop

    def stop(self):
        ctrlSvc = ControlService(self._connectionString)
        ctrlSvc.stop()
        isBusy()

    def shutdown(self):
        stop()
        scnSvc = ScannerService(self._connectionString)
        scnSvc.shutdown()
