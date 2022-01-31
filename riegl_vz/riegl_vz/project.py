import sys
import os
import time
from os.path import join, basename

from rclpy.node import Node

from vzi_services.projectservice import ProjectService
from vzi_services.dataprocservice import DataprocService
from .ssh import RemoteClient

class RieglVzProject():
    def __init__(self, node):
        self._node = node
        self._hostname = node.hostname
        self._connectionString = self._hostname + ":20000"
        self._hostname = node.hostname
        self._sshUser = node.sshUser
        self._sshPwd = node.sshPwd
        self._logger = node.get_logger()

    def loadProject(self, projectName: str, storageMedia: int):
        try:
            projSvc = ProjectService(self._connectionString)
            projSvc.setStorageMedia(storageMedia)
            projSvc.loadProject(projectName);
        except:
            self._logger.error("Loading of project '{}' failed!".format(projectName))
            return False
        return True

    def createProject(self, projectName: str, storageMedia: int):
        try:
            projSvc = ProjectService(self._connectionString)
            projSvc.setStorageMedia(storageMedia)
            projSvc.createProject(projectName)
            projSvc.loadProject(projectName);
        except:
            self._logger.error("Creating project '{}' failed!".format(projectName))
            return False
        return True

    def getActiveScanposPath(self, scanposName: str):
        return self._getActiveProjectPath() + '/' + scanposName + '.SCNPOS'

    def getActiveProjectPath(self):
        projSvc = ProjectService(self._connectionString)
        return projSvc.projectPath()

    def _getProjectPath(self, projectName: str, storageMedia: int):
        projSvc = ProjectService(self._connectionString)
        projectPath = projSvc.projectPath(storageMedia, projectName)
        return projectPath

    def _getCurrentScanpos(self, projectName: str, storageMedia: int):
        self._logger.debug("get next scanpos: projectName={}, storageMedia={}".format(projectName, storageMedia))
        ssh = RemoteClient(host=self._hostname, user=self._sshUser, password=self._sshPwd)
        cmd = ["ls -1", self._getProjectPath(projectName, storageMedia), " | sort -n", " | grep '.SCNPOS'", " | sed 's/.SCNPOS//g'", " | tail -n 1"]
        self._logger.debug("CMD = {}".format(" ".join(cmd)))
        response = ssh.executeCommand(" ".join(cmd))
        self._logger.debug("RESP = {}".format(" ".join(response)))
        ssh.disconnect()

        if len(response) == 0:
            return 0
        return int(response[0])

    def getCurrentScanpos(self, projectName: str, storageMedia: int):
        scanpos = self._getCurrentScanpos(projectName, storageMedia)
        return str(scanpos) if (scanpos > 0) else ""

    def getNextScanpos(self, projectName: str, storageMedia: int):
        return str(self._getCurrentScanpos(projectName, storageMedia) + 1)

    def getScanId(self, scanposName: str):
        if int(scanposName) == 0:
            procSvc = DataprocService(self._connectionString)
            return procSvc.actualFile(0)

        ssh = RemoteClient(host=self._hostname, user=self._sshUser, password=self._sshPwd)
        scanposPath = self.getActiveScanposPath(scanposName) + '/scans'
        cmd = ["ls -t", scanposPath + "/*.rxp"]
        self._logger.debug("CMD = {}".format(" ".join(cmd)))
        response = ssh.executeCommand(" ".join(cmd))
        self._logger.debug("RESP = {}".format(" ".join(response)))
        ssh.disconnect()

        if len(response) == 0:
            return "null"

        return (scanposPath + "/" + os.path.basename(response[0]).split(".")[0] + ".rxp").replace("/media/", "")
