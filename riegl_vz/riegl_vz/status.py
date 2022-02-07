import time
import json
import math
import threading

from vzi_services.scannerservice import ScannerService
from vzi_services.controlservice import ControlService
from vzi_services.interfaceservice import InterfaceService
from vzi_services.gnssbaseservice import GnssBaseService
from vzi_services.riconnectswitch import RiconnectSwitch

class ScannerStatus(object):
    def __init__(self):
        self.err = False
        self.instIdent = ""
        self.serialNumber = ""
        self.opstate = "unavailable"
        self.activeTask = ""
        self.progress = 0

class MemoryStatus(object):
    def __init__(self):
        self.err = False
        self.memTotalGB = 0
        self.memFreeGB = 0
        self.memUsage = 0

class GnssStatus(object):
    def __init__(self):
        self.err = False
        self.fix = True
        self.numSat = 3
        self.longitude = 15.656631
        self.latitude = 48.6625
        self.altitude = math.nan

class ErrorStatus(object):
    def __init__(self):
        self.err = False
        self.numErrors = 0
        self.numWarnings = 0

class StatusMaintainer(object):
    def __init__(self):
        self._scannerStatus: ScannerStatus = ScannerStatus()
        self._memoryStatus: MemoryStatus = MemoryStatus()
        self._gnssStatus: GnssStatus = GnssStatus()
        self._errorStatus: ErrorStatus = ErrorStatus()
        self._threadLock = threading.Lock()

    def _lock(self):
        self._threadLock.acquire()

    def _unlock(self):
        self._threadLock.release()

    def setOpstate(self, opstate, task = ""):
        self._lock()
        self._scannerStatus.opstate = opstate
        self._scannerStatus.activeTask = task
        self._unlock()

    def setActiveTask(self, task):
        self._lock()
        self._scannerStatus.activeTask = task
        self._unlock()

    def setProgress(self, progress):
        self._lock()
        self._scannerStatus.progress = progress
        self._unlock()

    def getScannerStatus(self):
        self._lock()
        status = self._scannerStatus
        self._unlock()
        return status

    def setMemoryStatus(self, status):
        self._lock()
        self._memoryStatus = status
        self._unlock()

    def getMemoryStatus(self):
        self._lock()
        status = self._memoryStatus
        self._unlock()
        return status

    def setGnssStatus(self, status):
        self._lock()
        self._gnssStatus = status
        self._unlock()

    def getGnssStatus(self):
        self._lock()
        status = self._gnssStatus
        self._unlock()
        return status

    def setErrorStatus(self, status):
        self._lock()
        self._errorStatus = status
        self._unlock()

    def getErrorStatus(self):
        self._lock()
        status = self._errorStatus
        self._unlock()
        return status

class RieglVzStatus():
    def __init__(self, node):
        self._node = node
        self._logger = node.get_logger()
        self._hostname = node.hostname
        self._connectionString = self._hostname + ":20000"
        self._node.storageMedia = 0
        self._ctrlSvc = None
        self._scanSvc = None
        self._intfSvc = None
        self._gnssSvc = None
        self._riconSw = None
        self._shutdownReq = False

        self._detectThread = threading.Thread(target=self._detectFunction, args=())
        self._detectThread.daemon = True
        self._detectThread.start()

        self.status: StatusMaintainer = StatusMaintainer()
        self.trigStarted = False

    def _detectFunction(self):
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
            except:
                pass
            time.sleep(1.0)

        if not self._shutdownReq:
            try:
                self._scanSvc = ScannerService(self._connectionString)
                ok, instInfoErr, instIdent, serialNumber = self._getInstInfo()
                if ok:
                    self.status._scannerStatus.instIdent = instIdent
                    self.status._scannerStatus.serialNumber = serialNumber
                    self._logger.info("{} {} is available now!".format(instIdent, serialNumber))
            except:
                self.status._scannerStatus.err = True
                self._logger.error("ScannerService is not available!")
            try:
                self._intfSvc = InterfaceService(self._connectionString)
            except:
                self.status._memoryStatus.err = True
                self._logger.error("InterfaceService is not available!")
            try:
                self._gnssSvc = GnssBaseService(self._connectionString)
            except:
                self.status._gnssStatus.err = True
                self._logger.error("GnssBaseService is not available!")
            try:
                self._riconSw = RiconnectSwitch(self._connectionString)
            except:
                self.status._errorStatus.err = True
                self._logger.error("RiconnectSwitch is not available!")

        self._shutdownReq = False

        self._memoryStatusCallback()
        self._node.create_timer(10.0, self._memoryStatusCallback)
        self._gnssStatusCallback()
        self._node.create_timer(1.0, self._gnssStatusCallback)

    def _getInstInfo(self):
        ok = False
        err = False
        instIdent = ""
        serialNumber = ""
        if self._scanSvc:
            ok = True
            instIdent = self._scanSvc.instrumentInformation().identifier
            serialNumber = self._scanSvc.instrumentInformation().serialNumber
        return ok, err, instIdent, serialNumber

    def getScannerStatus(self):
        return self.status.getScannerStatus()

    def getScannerOpstate(self):
        return self.status.getScannerStatus().opstate

    def isScannerAvailable(self):
        return (self.getScannerOpstate() != "unavailable")
        
    def _memoryStatusCallback(self):
        memoryStatus = MemoryStatus()
        try:
            if self._intfSvc:
                intf = 0
                if self._node.storageMedia == 0:
                    intf = self._intfSvc.STORAGEIF_SSD
                elif self._node.storageMedia == 1:
                    intf = self._intfSvc.STORAGEIF_USB
                elif self._node.storageMedia == 2:
                    intf = self._intfSvc.STORAGEIF_SDCARD
                storageIfs = self._intfSvc.getStorageInterfaces(intf)
                if len(storageIfs) > 0:
                    totalSpace = storageIfs[0]["mounts"][0]["storage_space"]["total_space"]
                    usedSpace = storageIfs[0]["mounts"][0]["storage_space"]["used_space"]
                    memoryStatus.memTotalGB = totalSpace / 1024.0 / 1024.0
                    memoryStatus.memFreeGB = (totalSpace - usedSpace) / 1024.0 / 1024.0
                    memoryStatus.memUsage = usedSpace / totalSpace * 100.0
                else:
                    memoryStatus.err = True
            else:
                memoryStatus.err = True
        except:
            memoryStatus.err = True
        self.status.setMemoryStatus(memoryStatus)

    def getMemoryStatus(self):
        return self.status.getMemoryStatus()

    def _gnssStatusCallback(self):
        gnssStatus = GnssStatus()
        try:
            if self._gnssSvc:
                j = json.loads(self._gnssSvc.estimateInfo())
                #self._logger.debug("gnss: {}".format(j))
                if "fix" in j:
                    gnssStatus.fix = j["fix"]
                if "num_sat" in j:
                    gnssStatus.numSat = j["num_sat"]
                if "longitude" in j:
                    gnssStatus.longitude = j["longitude"]
                if "latitude" in j:
                    gnssStatus.latitude = j["latitude"]
                if "height" in j:
                    gnssStatus.altitude = j["height"]
        except:
            gnssStatus.err = True
        self.status.setGnssStatus(gnssStatus)

    def getGnssStatus(self):
        return self.status.getGnssStatus()

    def _errorStatusCallback(self):
        errorStatus = ErrorStatus()
        try:
            if self._riconSw:
                sysErrors = self._riconSw.readSysErrors()
                for sysError in sysErrors:
                    if sysError.severity == 0:
                        errorStatus.numWarnings += 1
                    else:
                        errorStatus.numErrors += 1
        except:
            errorStatus.err = True
        self.status.setErrorStatus(errorStatus)

    def getErrorStatus(self):
        return self.status.getErrorStatus()

    def shutdown(self):
        self._shutdownReq = True
        while self.shutdownReq:
            time.sleep(0.1)
