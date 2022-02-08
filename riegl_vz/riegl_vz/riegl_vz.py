import sys
import os
import time
import json
from datetime import datetime
import subprocess
import threading
import numpy as np
from os.path import join, dirname, basename, abspath

from std_msgs.msg import (
    Header
)
from sensor_msgs.msg import (
    PointCloud2,
    PointField,
    NavSatStatus,
    NavSatFix
)
from geometry_msgs.msg import (
    PoseWithCovariance,
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
from vzi_services.interfaceservice import InterfaceService
from vzi_services.projectservice import ProjectService
from vzi_services.scannerservice import ScannerService

from .pose import (
    readVop,
    readPop,
    readAllSopv,
    getTransformFromPose
)
from .project import RieglVzProject
from .status import RieglVzStatus
from .ssh import RieglVzSSH
from .utils import (
    SubProcess,
    parseCSV
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

class RieglVz():
    def __init__(self, node):
        self._node = node
        self._logger = node.get_logger()
        self._hostname = node.hostname
        self._workingDir = node.workingDir
        self._connectionString = self._hostname + ":20000"
        self._path = Path()
        self._position = None
        self._stopReq = False
        self._shutdownReq = False
        self._project: RieglVzProject = RieglVzProject(self._node)
        self._status: RieglVzStatus = RieglVzStatus(self._node)
        self._ssh: RieglVzSSH = RieglVzSSH(self._node)

        self.scanposName = None

        self.scanPublishFilter = node.scanPublishFilter
        self.scanPublishLOD = node.scanPublishLOD

        if not os.path.exists(self._workingDir):
            os.mkdir(self._workingDir)

    def _broadcastTfTransforms(self, ts: datetime.time):
        ok, pop = self.getPop()
        if ok:
            self._node.transformBroadcaster.sendTransform(getTransformFromPose(ts, "riegl_vz_prcs", pop))
        ok, vop = self.getVop()
        if ok:
            self._node.transformBroadcaster.sendTransform(getTransformFromPose(ts, "riegl_vz_vocs", vop))
            ok, sopv = self.getSopv()
            if ok:
                self._node.transformBroadcaster.sendTransform(getTransformFromPose(ts, "riegl_vz_socs", sopv.pose))
            else:
                return False, None
        else:
            return False, None
        return True, sopv

    def getScannerStatus(self):
        return self._status.getScannerStatus()

    def isScannerAvailable(self):
        return self._status.isScannerAvailable()

    def getScannerOpstate(self):
        return self.getScannerStatus().opstate

    def getMemoryStatus(self):
        return self._status.getMemoryStatus()

    def getGnssStatus(self):
        return self._status.getGnssStatus()

    def getErrorStatus(self):
        return self._status.getErrorStatus()

    def getCameraStatus(self):
        return self._status.getCameraStatus()

    def loadProject(self, projectName: str, storageMedia: int, scanRegister: bool):
        ok = self._project.loadProject(projectName, storageMedia)
        if ok and scanRegister:
            ts = self._node.get_clock().now()
            self._broadcastTfTransforms(ts)
        return ok;

    def createProject(self, projectName: str, storageMedia: int):
        if self._project.createProject(projectName, storageMedia):
            self._path = Path();
            return True
        return False

    def getCurrentScanpos(self, projectName: str, storageMedia: int):
        return self._project.getCurrentScanpos(projectName, storageMedia);

    def getNextScanpos(self, projectName: str, storageMedia: int):
        return self._project.getNextScanpos(projectName, storageMedia)

    #def _getTimeStampFromScanId(self, scanId: str):
    #    scanFileName: str = os.path.basename(scanId)
    #    dateTime = datetime.strptime(scanFileName, '%y%m%d_%H%M%S.rxp')
    #    #self._logger.debug("dateTime = {}".format(dateTime))
    #    return int(dateTime.strftime("%s"))

    def _setPositionEstimate(self, position):
        scanposPath = self._project.getActiveScanposPath(self.scanposName)
        remoteFile = scanposPath + "/final.pose"
        localFile = self._workingDir + "/final.pose"
        self._ssh.downloadFile(remoteFile, localFile)
        with open(localFile, "r") as f:
            finalPose = json.load(f)
        finalPose["positionEstimate"] = {
            "coordinateSystem": position.header.frame_id,
            "coord1": position.point.x,
            "coord2": position.point.y,
            "coord3": position.point.z
        }
        with open(localFile, "w") as f:
            json.dump(finalPose, f, indent=4)
            f.write("\n")
        self._ssh.uploadFile([localFile], scanposPath)

    def _getGnssFixMessage(self):
        status = self._status.getGnssStatus()

        if not status.valid:
            return False, None

        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "riegl_vz_gnss"

        if status.fix:
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position in degrees.
        msg.latitude = float(status.latitude)
        msg.longitude = float(status.longitude)

        # Altitude in metres.
        msg.altitude = float(status.altitude)

        msg.position_covariance[0] = 0
        msg.position_covariance[4] = 0
        msg.position_covariance[8] = 0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        return True, msg

    def publishGnssFix(self):
        ok, msg = self._getGnssFixMessage()
        if ok:
            self._node.gnssFixPublisher.publish(msg)

    def publishScanGnssFix(self):
        ok, msg = self._getGnssFixMessage()
        if ok:
            self._node.scanGnssFixPublisher.publish(msg)

    def setProjectControlPoints(coordSystem: str, csvFile: str):
        projectPath = self._project.getActiveProjectPath()
        remoteSrcCpsFile = csvFile
        localSrcCpsFile = self._workingDir + "/" + os.path.basename(csvFile)
        self._ssh.downloadFile(remoteSrcCpsFile, localSrcCpsFile)
        csvData = parseCSV(localSrcCpsFile)[1:]
        # parse points and write resulting csv file
        controlPoints = []
        if csvData:
            if len(csvData[0]) < 4:
                raise RuntimeError("Invalid control points definition. File must have at least four columns.")
            localDstCpsFile = self._workingDir + "/controlpoints.csv"
            with open(localDstCpsFile, "w") as f:
                f.write("Name,CRS,Coord1,Coord2,Coord3\n")
                for item in csvData:
                    f.write("{},{},{},{},{}\n".format(item[0], coordSystem, item[1], item[2], item[3]))
            self._ssh.uploadFile([localDstCpsFile], projectPath)

    def getPointCloud(self, scanposName: str, pointcloud: PointCloud2, ts: bool = True):
        self._logger.debug("Downloading rdbx file..")
        self._status.status.setActiveTask("download rdbx file")
        scanId = self._project.getScanId(scanposName)
        self._logger.debug("scan id = {}".format(scanId))
        if scanId == "null":
            self._logger.error("Scan id is null!")
            return False, pointcloud

        scanposPath = self._project.getActiveScanposPath(scanposName)
        self._logger.debug("scanpos path = {}".format(scanposPath))
        scan = os.path.basename(scanId).replace(".rxp", "")[0:13]
        self._logger.debug("scan = {}".format(scan))
        remoteFile = scanposPath + "/scans/" + scan + ".rdbx"
        localFile = self._workingDir + "/scan.rdbx"
        self._ssh.downloadFile(remoteFile, localFile)

        self._logger.debug("Generate point cloud..")
        self._status.status.setActiveTask("generate point cloud data")
        with riegl.rdb.rdb_open(localFile) as rdb:
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

            if ts:
                stamp = self._node.get_clock().now().to_msg()
            else:
                stamp = builtin_msgs.Time(sec = 0, nanosec = 0)

            header = std_msgs.Header(frame_id = "riegl_vz_socs", stamp = stamp)

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
        self._status.status.setActiveTask("")
        self._logger.debug("Point cloud generated.")

        return True, pointcloud

    def _scanThreadFunc(self):
        self._status.status.setOpstate("scanning", "scan data acquisition")
        self._status.status.setProgress(0)

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
        if self.reflSearchSettings:
            self._logger.info("reflector search = {}".format(self.reflSearchSettings))
        self._logger.info("image capture = {}".format(self.captureImages))
        self._logger.info("image capture mode = {}".format(self.captureMode))
        self._logger.info("image capture overlap = {}".format(self.imageOverlap))

        scriptPath = join(appDir, "acquire-data.py")
        cmd = [
            "python3", scriptPath,
            "--connectionstring", self._connectionString,
            "--project", self.projectName,
            "--scanposition", self.scanposName,
            "--storage-media", str(self.storageMedia)]
        if self.reflSearchSettings:
            rssFilePath = join(self._workingDir, "reflsearchsettings.json")
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
            self._status.status.setOpstate("waiting")
            self._logger.info("Scan stopped")
            return
        self._logger.info("Data acquisition finished")

        self._logger.info("Publish scan position GNSS fix..")
        self.publishScanGnssFix()

        self._status.status.setOpstate("processing")

        self._logger.info("Converting RXP to RDBX..")
        self._status.status.setActiveTask("convert rxp to rdbx")
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
            self._status.status.setOpstate("waiting")
            self._logger.info("Scan stopped")
            return
        self._logger.info("RXP to RDBX conversion finished")

        if self._position is not None:
            self._status.status.setActiveTask("set position estimate")
            if self.scanposName == '1':
                self._logger.info("Set project position.")
                projSvc = ProjectService(self._connectionString)
                projSvc.setProjectLocation(self._position.header.frame_id, self._position.point.x, self._position.point.y, self._position.point.z)
            self._logger.info("Set scan position estimate..")
            try:
                self._setPositionEstimate(self._position)
                self._logger.info("Set position estimate finished")
            except:
                self._logger.error("Set position estimate failed!")
            self._position = None

        if self.scanRegister:
            self._logger.info("Starting registration..")
            self._status.status.setActiveTask("scan position registration")
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
                self._status.status.setOpstate("waiting")
                self._logger.info("Scan stopped")
                return
            self._logger.info("Registration finished")

            self._logger.info("Downloading and publishing pose..")
            self._status.status.setActiveTask("publish registered scan position")
            ts = self._node.get_clock().now()
            ok, sopv = self._broadcastTfTransforms(ts)
            if ok:
                # update sopv timestamp
                sopv.pose.header.stamp = ts.to_msg()
                # publish pose
                self._node.posePublisher.publish(sopv.pose)
                # publish path
                self._path.header = sopv.pose.header
                self._path.poses.append(sopv.pose)
                self._node.pathPublisher.publish(self._path)
                # publish odometry
                odom = Odometry(
                    header = Header(stamp = ts.to_msg(), frame_id = "riegl_vz_vocs"),
                    child_frame_id = "riegl_vz_socs",
                    pose = PoseWithCovariance(pose = sopv.pose.pose)
                )
                self._node.odomPublisher.publish(odom)
            self._logger.info("Pose published")

        if self.scanPublish:
            self._logger.info("Downloading and publishing point cloud..")
            pointcloud: PointCloud2 = PointCloud2()
            ts = self._node.get_clock().now()
            ok, pointcloud = self.getPointCloud(self.scanposName, pointcloud)
            if ok:
                self._status.status.setActiveTask("publish point cloud data")
                self._node.pointCloudPublisher.publish(pointcloud)
            self._logger.info("Point cloud published")

        self._status.status.setOpstate("waiting")

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
        captureImages: bool = True,
        captureMode: int = 1,
        imageOverlap: int = 25):
        """Acquire data at scan position.

        Args:
          projectName ... the project name
          scanposName ... the name of the new scan position
          storageMedia ... storage media for data recording
          scanPattern ... the scan pattern
          reflSearchSettings ... reflector search settings"""

        if self.isBusy(block=False):
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
            while self.getScannerOpstate() == "scanning":
                time.sleep(0.2)
        return True if self.getScannerOpstate() == "scanning" else False

    def isBusy(self, block = True):
        if block:
            while self.getScannerOpstate() != "waiting":
                time.sleep(0.2)
        return False if self.getScannerOpstate() == "waiting" else True

    def setPosition(self, position):
        self._position = position

    def getAllSopv(self):
        try:
            sopvFileName = "all_sopv.csv"
            remoteFile = self._project.getActiveProjectPath() + "/Voxels1.VPP/" + sopvFileName
            localFile = self._workingDir + "/" + sopvFileName
            self._ssh.downloadFile(remoteFile, localFile)
            ok = True
            sopvs = readAllSopv(localFile, self._logger)
        except Exception as e:
            ok = False
            sopvs = None

        return ok, sopvs

    def getSopv(self):
        ok, sopvs = self.getAllSopv()
        if ok and len(sopvs):
            sopv = sopvs[-1]
        else:
            sopv = None
            ok = False

        return ok, sopv

    def getVop(self):
        try:
            sopvFileName = "VPP.vop"
            remoteFile = self._project.getActiveProjectPath() + "/Voxels1.VPP/" + sopvFileName
            localFile = self._workingDir + "/" + sopvFileName
            self._ssh.downloadFile(remoteFile, localFile)
        except Exception as e:
            return False, None

        vop = readVop(localFile)

        return True, vop

    def getPop(self):
        try:
            popFileName = "project.pop"
            remoteFile = self._project.getActiveProjectPath() + "/" + popFileName
            localFile = self._workingDir + "/" + popFileName
            self._ssh.downloadFile(remoteFile, localFile)
        except Exception as e:
            return False, None

        pop = readPop(localFile)

        return True, pop

    def stop(self):
        self._stopReq = True

        if self.isScannerAvailable():
            ctrlSvc = ControlService(self._connectionString)
            ctrlSvc.stop()
            self.isBusy()

    def trigStartStop(self):
        trigStartedPrev = self._status.trigStarted
        if not self._status.trigStarted:
            if self.isBusy(block = False):
                return False
            self._status.trigStarted = True

        intfSvc = InterfaceService(self._connectionString)
        intfSvc.triggerInputEvent("ACQ_START_STOP")

        if not trigStartedPrev and self._status.trigStarted:
            startTime = time.time()
            while not self.isScanning(block=False):
                time.sleep(0.2)
                if (time.time() - startTime) > 5:
                    self._status.trigStarted = False
                    return False

        return True

    def getScanPatterns(self):
        patterns: str = []
        ctrlSvc = ControlService(self._connectionString)
        for pattern in json.loads(ctrlSvc.scanPatternsDetailed()):
            patterns.append(pattern["name"])
        #instIdentLower = self._status.status.scannerStatus.instIdent.lower()
        #remotePath = "/usr/share/gui/" + instIdentLower + "/patterns"
        #files = self._ssh.listFiles(remotePath, "*.pat")
        #for file in files:
        #    patterns.append(os.path.basename(file).replace(".pat", ""))
        return True, patterns

    def getScanPattern(self, patternName):
        ctrlSvc = ControlService(self._connectionString)
        for p in json.loads(ctrlSvc.scanPatternsDetailed()):
            if p["name"] == patternName:
                pattern: ScanPattern = ScanPattern()
                pattern.lineStart = p["thetaStart"]
                pattern.lineStop = p["thetaStop"]
                pattern.lineIncrement = p["thetaIncrement"]
                pattern.frameStart = p["phiStart"]
                pattern.frameStop = p["phiStop"]
                pattern.frameIncrement = p["phiIncrement"]
                return True, pattern
        self._logger.error("Scan pattern '{}' is not available!")
        return False, None

    def getReflectorModels(self):
        models: str = []
        ctrlSvc = ControlService(self._connectionString)
        for model in json.loads(ctrlSvc.supportedReflectorSearchModels()):
            models.append(model["name"])
        return True, models

    def shutdown(self):
        self._status.shutdown()
        self.stop()
        if self.isScannerAvailable():
            scnSvc = ScannerService(self._connectionString)
            scnSvc.shutdown()
