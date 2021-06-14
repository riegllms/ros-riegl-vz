import sys
import os
import subprocess
from os.path import join, dirname, abspath

from rclpy.node import Node

from vzi_services.controlservice import ControlService
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
    def __init__(self, connectionString, workingDir, logger):
        self.connectionString = connectionString
        self.workingDir = workingDir
        self.logger = logger

    def acquireData(
        self,
        projectName: str,
        scanposName: str,
        scanPattern: ScanPattern,
        reflSearchSettings: dict = None,
        createRdbx: bool = False,
        block: bool = True,
        rdbxLineStep: int = 5,
        rdbxEchoStep: int = 5,
        captureImages: bool = False,
        captureMode: int = 1,
        imageOverlap: int = 25) -> subprocess.Popen:
        """Acquire data at scan position.

        Args:
          projectName ... the project name
          scanposName ... the name of the new scan position
          scanPattern ... the scan pattern
          reflSearchSettings ... reflector search settings
          block ... block until done

        Return:
          Instance of subprocess.Popen if in non-blocking mode."""

        print("Acquiring data", flush=True)
        self.logger.info("Starting data acquisition")
        scriptPath = join(appDir, "bin", "acquire-data.py")
        cmd = [
            "python3", scriptPath,
            "--connectionstring", self.connectionString,
            "--project", projectName,
            "--scanposition", scanposName]
        if createRdbx:
            cmd.extend([
                "--create-rdbx",
                "--rdbx-line-step", str(rdbxLineStep),
                "--rdbx-echo-step", str(rdbxEchoStep)
            ])
        if reflSearchSettings:
            rssFilepath = join(self.workingDir, "reflsearchsettings.json")
            with open(rssFilepath, "w") as f:
                json.dump(reflSearchSettings, f)
            cmd.append("--reflsearch")
            cmd.append(rssFilepath)
        if scanPattern:
            cmd.extend([
                "--line-start", str(scanPattern.lineStart),
                "--line-stop", str(scanPattern.lineStop),
                "--line-incr", str(scanPattern.lineIncrement),
                "--frame-start", str(scanPattern.frameStart),
                "--frame-stop", str(scanPattern.frameStop),
                "--frame-incr", str(scanPattern.frameIncrement),
                "--measprog", str(scanPattern.measProgram)
            ])
        if captureImages:
            cmd.extend([
                "--capture-images",
                "--capture-mode", str(captureMode),
                "--image-overlap", str(imageOverlap)
            ])
        self.logger.debug("CMD = {}".format(" ".join(cmd)))
        proc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        if block:
            proc.waitFor("Data acquisition failed.")
            self.logger.info("Data acquisition finished")
            return None
        else:
            return proc

    def registerScan(
        self,
        projectName: str,
        scanposName: str,
        block: bool = True) -> subprocess.Popen:

        print("Registering", flush=True)
        self.logger.info("Starting registration")
        scriptPath = os.path.join(appDir, "bin", "register-scan.py")
        cmd = [
            "python3", scriptPath,
            "--project", projectName,
            "--scanposition", scanposName]
        self.logger.debug("CMD = {}".format(" ".join(cmd)))
        proc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        if block:
            proc.waitFor("Registration failed.")
            self.logger.info("Registration finished")
            return None
        else:
            return proc

    def coarsePose(self):
        # tbd...
        return

    def status(self):
        # tbd...
        return

    def stop(self):
        ctrlSvc = ControlService(self.connectionString)
        ctrlSvc.stop()

    def shutdown(self):
        ctrlSvc = ControlService(self.connectionString)
        scnSvc = ScannerService(self.connectionString)
        ctrlSvc.stop()
        scnSvc.shutdown()
