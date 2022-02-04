import time
import json
import threading

from vzi_services.controlservice import ControlService
from vzi_services.interfaceservice import InterfaceService
from vzi_services.gnssbaseservice import GnssBaseService

class ScannerStatus(object):
    def __init__(self):
        self.err = False
        self.opstate = "unavailable"
        self.progress = 0
        self.memTotalGB = 0
        self.memFreeGB = 0
        self.memUsage = 0

class GnssStatus(object):
    def __init__(self):
        self.err = False
        self.fix = False
        self.numSat = 0

class StatusMaintainer(object):
    def __init__(self):
        self.scannerStatus: ScannerStatus = ScannerStatus()
        self.gnssStatus: GnssStatus = GnssStatus()
        self._threadLock = threading.Lock()

    def _lock(self):
        self._threadLock.acquire()

    def _unlock(self):
        self._threadLock.release()

    def setOpstate(self, opstate):
        self._lock()
        self.scannerStatus.opstate = opstate
        self._unlock()


    def setProgress(self, progress):
        self._lock()
        self.scannerStatus.progress = progress
        self._unlock()

    def getScannerStatus(self):
        status: ScannerStatus
        self._lock()
        status = self.scannerStatus
        self._unlock()
        return status

class RieglVzStatus():
    def __init__(self, node):
        self._node = node
        self._logger = node.get_logger()
        self._hostname = node.hostname
        self._connectionString = self._hostname + ":20000"
        self._ctrlSvc = None
        self._intfSvc = None
        self._gnssSvc = None
        self._shutdownReq = False

        self._statusThread = threading.Thread(target=self._statusThreadFunc, args=())
        self._statusThread.daemon = True
        self._statusThread.start()

        self.status: StatusMaintainer = StatusMaintainer()
        self.trigStarted = False

    def _statusThreadFunc(self):
        def onTaskProgress(arg0):
            obj = json.loads(arg0)
            self._logger.debug("scan progress: {0} % ({1}, {2})".format(obj["progress"], obj["id"], obj["progresstext"]))
            if obj["id"] == 1:
                self.status.setProgress(obj["progress"])
            self._node._statusUpdater.force_update()

        def onDataAcquisitionStarted(arg0):
            if self.trigStarted:
                self.status.setOpstate("scanning")
                self._node._statusUpdater.force_update()
                self._logger.debug("Data Acquisition Started!")

        def onDataAcquisitionFinished(arg0):
            if self.trigStarted:
                self.status.setOpstate("waiting")
                self.trigStarted = False
                self._node._statusUpdater.force_update()
                self._logger.debug("Data Acquisition Finished!")

        while self._ctrlSvc is None and not self._shutdownReq:
            try:
                self._ctrlSvc = ControlService(self._connectionString)
                self._taskProgress = self._ctrlSvc.taskProgress().connect(onTaskProgress)
                self._acqStartedSigcon = self._ctrlSvc.acquisitionStarted().connect(onDataAcquisitionStarted)
                self._acqFinishedSigcon = self._ctrlSvc.acquisitionFinished().connect(onDataAcquisitionFinished)
                self.status.setOpstate("waiting")
                self._logger.debug("Scanner is available.")
            except:
                self._logger.debug("Scanner is not available!")
            time.sleep(1.0)

        if not self._shutdownReq:
            try:
                self._intfSvc = InterfaceService(self._connectionString)
            except:
                self._logger.error("InterfaceService is not available!")
            try:
                self._gnssSvc = GnssBaseService(self._connectionString)
            except:
                self.status.gnssStatus.err = True
                self._logger.error("GnssBaseService is not available!")

        self._shutdownReq = False

    def _getScannerMemUsage(self, storageMedia):
        err = False
        memTotalGB = 0
        memFreeGB = 0
        memUsage = 0
        if self._intfSvc:
            intf = 0
            if storageMedia == 0:
                intf = self._intfSvc.STORAGEIF_SSD
            elif storageMedia == 1:
                intf = self._intfSvc.STORAGEIF_USB
            elif storageMedia == 2:
                intf = self._intfSvc.STORAGEIF_SDCARD
            storageIfs = self._intfSvc.getStorageInterfaces(intf)
            if len(storageIfs) > 0:
                totalSpace = storageIfs[0]["mounts"][0]["storage_space"]["total_space"]
                usedSpace = storageIfs[0]["mounts"][0]["storage_space"]["used_space"]
                memTotalGB = totalSpace / 1024.0 / 1024.0
                memFreeGB = (totalSpace - usedSpace) / 1024.0 / 1024.0
                memUsage = usedSpace / totalSpace * 100.0
            else:
                err = True
        return err, memTotalGB, memFreeGB, memUsage

    def getScannerStatus(self, storageMedia):
        self.status.scannerStatus.err, memTotalGB, memFreeGB, memUsage = self._getScannerMemUsage(storageMedia)
        self.status.scannerStatus.memTotalGB = memTotalGB
        self.status.scannerStatus.memFreeGB = memFreeGB
        self.status.scannerStatus.memUsage = memUsage
        return self.status.getScannerStatus()

    def getScannerOpstate(self):
        return self.status.getScannerStatus().opstate

    def isScannerAvailable(self):
        return (self.getScannerOpstate() != "unavailable")

    def getGnssStatus(self):
        if self._gnssSvc:
            j = json.loads(self._gnssSvc.estimateInfo())
            self.status.gnssStatus.fix = j["fix"]
            self.status.gnssStatus.numSat = j["num_sat"]
        return self.status.gnssStatus

    def shutdown(self):
        self._shutdownReq = True
        while self.shutdownReq:
            time.sleep(0.1)
