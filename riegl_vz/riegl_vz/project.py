import sys
import os
import time
from os.path import join, basename

from rclpy.node import Node

from vzi_services.projectservice import ProjectService
from vzi_services.dataprocservice import DataprocService
from .ssh import RieglVzSSH

class RieglVzProject():
    def __init__(self, node):
        self._node = node
        self._hostname = node.hostname
        self._connectionString = self._hostname + ':20000'
        self._logger = node.get_logger()

        self._ssh: RieglVzSSH = RieglVzSSH(self._node)

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

    def getProjectName(self):
        projectName = None
        try:
            projSvc = ProjectService(self._connectionString)
            projectName = projSvc.projectName()
        except:
            self._logger.error("Get project name failed!")
            return False, None
        return True, projectName

    def getScanposName(self, scanposition:str):
        path = 'ScanPos' + scanposition.zfill(3)
        #self._logger.info("getScanposName = {}".format(path))
        return path

    def getActiveScanposPath(self, scanposition: str):
        path = self.getActiveProjectPath() + '/' + self.getScanposName(scanposition) + '.SCNPOS'
        #self._logger.info("getActiveScanposPath = {}".format(path))
        return path

    def getActiveProjectPath(self):
        projSvc = ProjectService(self._connectionString)
        path = projSvc.projectPath()
        #self._logger.info("getActiveProjectPath = {}".format(path))
        return path

    def _getProjectPath(self, projectName: str, storageMedia: int):
        projSvc = ProjectService(self._connectionString)
        projectPath = projSvc.projectPath(storageMedia, projectName)
        return projectPath

    def _getCurrentScanpos(self, projectName: str, storageMedia: int):
        self._logger.debug("get next scanpos: projectName={}, storageMedia={}".format(projectName, storageMedia))
        cmd = ["ls -1", self._getProjectPath(projectName, storageMedia), " | grep -E '^ScanPos[0-9]+\.SCNPOS$'", " | sed 's/.SCNPOS//g'", " | sed 's/ScanPos//g'", " | sort -n", " | tail -n 1"]
        rc, response = self._ssh.executeCommand(' '.join(cmd))

        if len(response) == 0:
            return 0

        scanpos = 0
        try:
            scanpos = int(response[0].lstrip('0'))
        except:
            scanpos = 0

        return scanpos

    def getCurrentScanpos(self, projectName: str, storageMedia: int):
        scanpos = self._getCurrentScanpos(projectName, storageMedia)
        return str(scanpos) if (scanpos > 0) else ''

    def getNextScanpos(self, projectName: str, storageMedia: int):
        return str(self._getCurrentScanpos(projectName, storageMedia) + 1)

    def getScanId(self, scanposName: str):
        if int(scanposName) == 0:
            procSvc = DataprocService(self._connectionString)
            return procSvc.actualFile(0)

        scanposPath = self.getActiveScanposPath(scanposName) + '/scans'
        cmd = ["ls -t", scanposPath + "/*.rxp"]
        rc, response = self._ssh.executeCommand(' '.join(cmd))

        if len(response) == 0:
            return 'null'

        return (scanposPath + '/' + os.path.basename(response[0]).split('.')[0] + '.rxp').replace('/media/', '')
