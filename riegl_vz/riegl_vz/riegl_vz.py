import sys
import os
import time
import subprocess
import threading
import numpy as np
from os.path import join, dirname, abspath

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import builtin_interfaces.msg as builtin_msgs

from rclpy.node import Node

import riegl.rdb

from vzi_services.controlservice import ControlService
from vzi_services.dataprocservice import DataprocService

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

class RieglVz():
    def __init__(self, node):
        self.node = node
        self.hostname = node.hostname
        self.sshUser = node.sshUser
        self.sshPwd = node.sshPwd
        self.workingDir = node.workingDir
        self.logger = node.get_logger()
        self.connectionString = self.hostname + ":20000"
        self.busy = False
        self.scanBusy = False
        self.rdbxFileRemote = None
        self.rdbxFileLocal = None
        if not os.path.exists(self.workingDir):
            os.mkdir(self.workingDir)

    def setProject(self, projectName: str):
        """Create a scan projectName.

        Args:
          projectName ... the project name"""
        if self.busy:
            return False

        self.projectName = projectName
        self.logger.info("Create project '{}'..".format(self.projectName))
        
        projSvc = ProjectService(self.connectionString)
        projSvc.createProject(self.projectName)
        projSvc.loadProject(self.projectName)

    def downloadAndPublishScan(self):
        self.logger.info("Downloading RDBX..")
        procSvc = DataprocService(self.connectionString)
        scanId = procSvc.actualFile(0)
        self.logger.debug("scan id = {}".format(scanId))
        self.rdbxFileRemote = "/media/" + scanId.replace(".rxp", ".rdbx")
        self.logger.debug("remote rdbx file = {}".format(self.rdbxFileRemote))
        self.rdbxFileLocal = self.workingDir + "/scan.rdbx"
        self.logger.debug("local rdbx file  = {}".format(self.rdbxFileLocal))
        ssh = RemoteClient(host=self.hostname, user=self.sshUser, password=self.sshPwd)
        ssh.download_file(filepath=self.rdbxFileRemote, localpath=self.rdbxFileLocal)
        ssh.disconnect()
        self.logger.info("RDBX download finished")

        self.logger.info("Extracting and publishing point cloud..")
        with riegl.rdb.rdb_open(self.rdbxFileLocal) as rdb:
            ts = builtin_msgs.Time(sec = 0, nanosec = 0)
            filter = ""
            rosDtype = sensor_msgs.PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize

            numTotalPoints = 0
            numPoints = 0
            data = bytearray()
            for points in rdb.select(
                self.scanPublishFilter,
                chunk_size=100000
                ):
                pointStep = 2 ** self.scanPublishLOD
                for point in points:
                    if not (numTotalPoints % pointStep):
                        data.extend(point["riegl.xyz"].astype(dtype).tobytes())
                        data.extend(point["riegl.reflectance"].astype(dtype).tobytes())
                        numPoints += 1
                    numTotalPoints += 1

            fields = [sensor_msgs.PointField(
                name = n, offset = i*itemsize, datatype = rosDtype, count = 1)
                for i, n in enumerate('xyzr')]

            header = std_msgs.Header(frame_id = "RIEGL_SOCS", stamp = ts)

            pointCloud = sensor_msgs.PointCloud2(
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
            #    self.logger.debug("{0}".format(point.riegl_xyz))

            self.node.pointCloudPublisher.publish(pointCloud)

        self.logger.info("Point cloud published")

    def scanThread(self):
        self.busy = True

        self.scanBusy = True
        self.logger.info("Starting data acquisition..")
        scriptPath = join(appDir, "acquire-data.py")
        cmd = [
            "python3", scriptPath,
            "--connectionstring", self.connectionString,
            "--project", self.projectName,
            "--scanposition", self.scanposName]
        if self.reflSearchSettings:
            rssFilepath = join(self.workingDir, "reflsearchsettings.json")
            with open(rssFilepath, "w") as f:
                json.dump(self.reflSearchSettings, f)
            cmd.append("--reflsearch")
            cmd.append(rssFilepath)
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
        self.logger.debug("CMD = {}".format(" ".join(cmd)))
        subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self.logger.debug("Subprocess started.")
        subproc.waitFor("Data acquisition failed.")
        self.logger.info("Data acquisition finished")
        self.scanBusy = False

        self.logger.info("Converting RXP to RDBX..")
        scriptPath = join(appDir, "create-rdbx.py")
        cmd = [
            "python3", scriptPath,
            "--connectionstring", self.connectionString,
            "--project", self.projectName,
            "--scanposition", self.scanposName]
        self.logger.debug("CMD = {}".format(" ".join(cmd)))
        subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self.logger.debug("Subprocess started.")
        subproc.waitFor("RXP to RDBX conversion failed.")
        self.logger.info("RXP to RDBX conversion finished")

        if self.scanPublish:
            self.downloadAndPublishScan()

        if self.scanRegister:
            print("Registering", flush=True)
            self.logger.info("Starting registration")
            scriptPath = os.path.join(appDir, "bin", "register-scan.py")
            cmd = [
                "python3", scriptPath,
                "--project", self.projectName,
                "--scanposition", self.scanposName]
            self.logger.debug("CMD = {}".format(" ".join(cmd)))
            subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
            subproc.waitFor("Registration failed.")
            self.logger.info("Registration finished")

        self.busy = False

    def scan(
        self,
        projectName: str,
        scanposName: str,
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
          scanPattern ... the scan pattern"""
        if self.busy:
            return False

        self.projectName = projectName
        self.scanposName = scanposName
        self.scanPattern = scanPattern
        self.scanPublish = scanPublish
        self.scanPublishFilter = scanPublishFilter
        self.scanPublishLOD = scanPublishLOD
        self.scanRegister = scanRegister
        self.reflSearchSettings = reflSearchSettings
        self.captureImages = captureImages
        self.captureMode = captureMode
        self.imageOverlap = imageOverlap

        thread = threading.Thread(target=self.scanThread, args=())
        thread.daemon = True
        thread.start()

        return True

    def isScanBusy(self, block = True):
        if block:
            while self.scanBusy:
                time.sleep(0.2)
        return self.scanBusy

    def isBusy(self, block = True):
        if block:
            while self.busy:
                time.sleep(0.2)
        return self.busy

    def status(self):
        # tbd...
        return

    def stop(self):
        ctrlSvc = ControlService(self.connectionString)
        ctrlSvc.stop()
        isBusy()

    def shutdown(self):
        stop()
        scnSvc = ScannerService(self.connectionString)
        scnSvc.shutdown()
