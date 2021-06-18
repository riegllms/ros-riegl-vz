import sys
import os
import time
import subprocess
import threading
from os.path import join, dirname, abspath

from rclpy.node import Node

from vzi_services.controlservice import ControlService
from vzi_services.dataprocservice import DataprocService
from .ssh import RemoteClient
from .utils import (
    SubProcess
)

#def scp():
#    host = 'H2222273'
#    user = 'user'
#    password = 'user'
#    remote_path = '/media/intern/'
#    cli = RemoteClient(host=host, user=user, password=password)
#
#    cli.download_file(file="/media/intern/gsm_kolomela.gsfx")
#
#    cli.disconnect()

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
    def __init__(
        self,
        hostname: str,
        sshUser: str,
        sshPwd: str,
        workingDir: str,
        logger):
        self.hostname = hostname
        self.sshUser = sshUser
        self.sshPwd = sshPwd
        self.workingDir = workingDir
        self.logger = logger
        self.connectionString = hostname + ":20000"
        self.busy = False
        self.scanBusy = False

    def acquireDataThread(self):
        self.busy = True

        self.scanBusy = True
        self.logger.info("Starting data acquisition..")
        scriptPath = join(appDir, "acquire-data.py")
        cmd = [
            "python3", scriptPath,
            "--connectionstring", self.connectionString,
            "--project", self.projectName,
            "--scanposition", self.scanposName,
            "--line-stop", str(self.lineStep),
            "--echo-step", str(self.echoStep)]
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

        self.logger.info("Download RDBX..")
        procSvc = DataprocService(self.connectionString)
        scanId = procSvc.actualFile(0)
        rdbxFile = "/media/" + scanId + ".rdbx"
        self.logger.debug("rdbx file = {}".format(rdbxFile))
        ssh = RemoteClient(host=self.hostname, user=self.sshUser, password=self.sshPwd)
        ssh.download_file(file=rdbxFile)
        ssh.disconnect()
        self.logger.info("RDBX download finished")

        self.busy = False

    def acquireData(
        self,
        projectName: str,
        scanposName: str,
        scanPattern: ScanPattern,
        reflSearchSettings: dict = None,
        lineStep: int = 1,
        echoStep: int = 1,
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
        self.reflSearchSettings = reflSearchSettings
        self.lineStep = lineStep
        self.echoStep = echoStep
        self.captureImages = captureImages
        self.captureMode = captureMode
        self.imageOverlap = imageOverlap

        thread = threading.Thread(target=self.acquireDataThread, args=())
        thread.daemon = True
        thread.start()

        return True

    def registerScanThread(self):
        self.busy = True
        print("Registering", flush=True)
        self.logger.info("Starting registration")
        scriptPath = os.path.join(appDir, "bin", "register-scan.py")
        cmd = [
            "python3", scriptPath,
            "--project", projectName,
            "--scanposition", scanposName]
        self.logger.debug("CMD = {}".format(" ".join(cmd)))
        subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        subproc.waitFor("Registration failed.")
        self.logger.info("Registration finished")
        self.busy = False

    def registerScan(
        self,
        projectName: str,
        scanposName: str):
        if self.busy:
            return False

        self.projectName: projectName
        self.scanposName: scanposName
        self.scanPattern: scanPattern

        thread = threading.Thread(target=self.registerScanThread, args=())
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
