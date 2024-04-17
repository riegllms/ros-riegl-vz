import time
import json
import math
import threading

from .geosys import RieglVzGeoSys

from vzi_services.scannerservice import ScannerService
from vzi_services.controlservice import ControlService
from vzi_services.interfaceservice import InterfaceService
from vzi_services.gnssbaseservice import GnssBaseService
from vzi_services.riconnectswitch import RiconnectSwitch
from vzi_services.cameraservice import CameraService
from vzi_services.geosysservice import GeoSysService

class ScannerStatus(object):
    def __init__(self):
        self.err = False
        self.instIdent = ''
        self.serialNumber = ''
        self.opstate = 'unavailable'
        self.activeTask = ''
        self.taskErrors = ''
        self.progress = 0
        self.laserOn = False

class MemoryStatus(object):
    def __init__(self):
        self.valid = False
        self.err = False
        self.memTotalGB = 0
        self.memFreeGB = 0
        self.memUsage = 0

class GnssStatus(object):
    def __init__(self):
        self.valid = False
        self.err = False
        self.enabled = True
        self.publish = True
        self.fix = True
        self.numSat = 3
        self.longitude = 15.656631
        self.latitude = 48.6625
        self.altitude = math.nan
        self.horAcc = math.nan
        self.verAcc = math.nan
        self.hdop = math.nan
        self.vdop = math.nan
        self.pdop = math.nan
        self.cs = ''

class ErrorStatus(object):
    def __init__(self):
        self.valid = False
        self.err = False
        self.numErrors = 0
        self.numWarnings = 0

class CameraStatus(object):
    def __init__(self):
        self.valid = False
        self.err = False
        self.avail = False

class StatusMaintainer(object):
    def __init__(self):
        self._scannerStatus: ScannerStatus = ScannerStatus()
        self._memoryStatus: MemoryStatus = MemoryStatus()
        self._gnssStatus: GnssStatus = GnssStatus()
        self._errorStatus: ErrorStatus = ErrorStatus()
        self._cameraStatus: CameraStatus = CameraStatus()
        self._threadLock = threading.Lock()

    def _lock(self):
        self._threadLock.acquire()

    def _unlock(self):
        self._threadLock.release()

    def setOpstate(self, opstate, task = ''):
        self._lock()
        self._scannerStatus.opstate = opstate
        self._scannerStatus.activeTask = task
        self._unlock()

    def clearTaskErrors(self):
        self._lock()
        self._scannerStatus.taskErrors = ''
        self._unlock()

    def addTaskError(self, errorMsg):
        self._lock()
        if self._scannerStatus.taskErrors != '':
            self._scannerStatus.taskErrors += ','
        self._scannerStatus.taskErrors += errorMsg
        self._unlock()

    def setActiveTask(self, task):
        self._lock()
        self._scannerStatus.activeTask = task
        self._unlock()

    def setProgress(self, progress):
        self._lock()
        self._scannerStatus.progress = progress
        self._unlock()

    def setLaserOn(self, state):
        self._lock()
        self._scannerStatus.laserOn = state
        self._unlock()

    def getScannerStatus(self):
        self._lock()
        status = self._scannerStatus
        self._unlock()
        return status

    def setMemoryStatus(self, status):
        self._lock()
        status.valid = True
        self._memoryStatus = status
        self._unlock()

    def getMemoryStatus(self):
        self._lock()
        status = self._memoryStatus
        self._unlock()
        return status

    def setGnssStatus(self, status):
        self._lock()
        status.valid = True
        self._gnssStatus = status
        self._unlock()

    def getGnssStatus(self):
        self._lock()
        status = self._gnssStatus
        self._unlock()
        return status

    def setErrorStatus(self, status):
        self._lock()
        status.valid = True
        self._errorStatus = status
        self._unlock()

    def getErrorStatus(self):
        self._lock()
        status = self._errorStatus
        self._unlock()
        return status

    def setCameraStatus(self, status):
        self._lock()
        status.valid = True
        self._cameraStatus = status
        self._unlock()

    def getCameraStatus(self):
        self._lock()
        status = self._cameraStatus
        self._unlock()
        return status

class RieglVzStatus():
    def __init__(self, node):
        self._node = node
        self._logger = node.get_logger()
        self._hostname = node.hostname
        self._connectionString = self._hostname + ':20000'
        self._node.storageMedia = 0
        self._ctrlSvc = None
        self._scanSvc = None
        self._intfSvc = None
        self._gnssSvc = None
        self._camSvc = None
        self._riconSw = None
        self._geoSvc = None
        self._shutdownReq = False
        self._running = False
        self._gnssPosUpdateSigcon = None

        self._detectThread = threading.Thread(target=self._detectFunction, args=())
        self._detectThread.daemon = True
        self._detectThread.start()

        self.status: StatusMaintainer = StatusMaintainer()
        self.trigStarted = False

    def _detectFunction(self):
        def onTaskProgress(arg0):
            obj = json.loads(arg0)
            self._logger.debug("scan progress: {0} % ({1}, {2})".format(obj['progress'], obj['id'], obj['progresstext']))
            if obj['id'] == 1:
                self.status.setProgress(obj['progress'])
            self._node._statusUpdater.force_update()

        def onDataAcquisitionStarted(arg0):
            if self.trigStarted:
                self.status.setOpstate('scanning')
                self._node._statusUpdater.force_update()
                self._logger.debug("Data Acquisition Started!")

        def onDataAcquisitionFinished(arg0):
            if self.trigStarted:
                self.status.setOpstate('waiting')
                self.trigStarted = False
                self._node._statusUpdater.force_update()
                self._logger.debug("Data Acquisition Finished!")

        def onGnssPositionUpdate(arg0):
            posInfo = json.loads(arg0)
            gnssStatus = self._getGnssStatusFromPositionUpdate(posInfo)
            gnssStatus.valid = True
            self._node._rieglVz.publishGnssFix(gnssStatus)

        while self._ctrlSvc is None and not self._shutdownReq:
            try:
                self._ctrlSvc = ControlService(self._connectionString)
                self._taskProgress = self._ctrlSvc.taskProgress().connect(onTaskProgress)
                self._acqStartedSigcon = self._ctrlSvc.acquisitionStarted().connect(onDataAcquisitionStarted)
                self._acqFinishedSigcon = self._ctrlSvc.acquisitionFinished().connect(onDataAcquisitionFinished)
                self.status.setOpstate('waiting')
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
            if self._gnssSvc is not None:
                try:
                    ricSvc = RiconnectSwitch(self._connectionString)
                    vstr = ricSvc.getMessages('version','GnssBaseService')
                    self._logger.info("GnssBaseService version: {}".format(vstr[0].text))
                    v = vstr[0].text.split(".")
                    if int(v[0]) == 1 and int(v[1]) >= 11 and int(v[2]) >= 1:
                        self._gnssPosUpdateSigcon = self._gnssSvc.positionUpdate().connect(onGnssPositionUpdate)
                        self._logger.info("GnssBaseService with positionUpdate signal.")
                    else:
                        self._logger.info("GnssBaseService without positionUpdate signal!")
                        self._gnssPosUpdateSigcon = None
                except:
                    self._logger.info("GnssBaseService without positionUpdate signal!")
                    self._gnssPosUpdateSigcon = None
            try:
                self._camSvc = CameraService(self._connectionString)
            except:
                self.status._cameraStatus.err = True
                self._logger.error("CameraService is not available!")
            try:
                self._riconSw = RiconnectSwitch(self._connectionString)
            except:
                self.status._errorStatus.err = True
                self._logger.error("RiconnectSwitch is not available!")

        self._shutdownReq = False

        self._timer10sCallback()
        self._node.create_timer(10.0, self._timer10sCallback)
        self._timer1sCallback()
        self._node.create_timer(1.0, self._timer1sCallback)

        self._running = True

    def _getInstInfo(self):
        ok = False
        err = False
        instIdent = ''
        serialNumber = ''
        if self._scanSvc:
            ok = True
            instIdent = self._scanSvc.instrumentInformation().identifier
            serialNumber = self._scanSvc.instrumentInformation().serialNumber
        return ok, err, instIdent, serialNumber

    def _timer10sCallback(self):
        # memory status
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
                    totalSpace = storageIfs[0]['mounts'][0]['storage_space']['total_space']
                    usedSpace = storageIfs[0]['mounts'][0]['storage_space']['used_space']
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

    def _getGnssStatusFromPositionUpdate(self, posInfo):
        gnssStatus = GnssStatus()
        #self._logger.debug("gnss (signal): {}".format(posInfo))
        if 'fix' in posInfo and posInfo['fix'] != None:
            gnssStatus.fix = posInfo['fix']
        if 'num_sat' in posInfo and posInfo['num_sat'] != None:
            gnssStatus.numSat = posInfo['num_sat']
        if 'longitude' in posInfo and posInfo['longitude'] != None:
            gnssStatus.longitude = float(posInfo['longitude'])
        if 'latitude' in posInfo and posInfo['latitude'] != None:
            gnssStatus.latitude = float(posInfo['latitude'])
        if 'height' in posInfo and posInfo['height'] != None:
            gnssStatus.altitude = float(posInfo['height'])
        if 'horAcc' in posInfo and posInfo['horAcc'] != None:
            gnssStatus.horAcc = float(posInfo['horAcc'])
        if 'verAcc' in posInfo and posInfo['verAcc'] != None:
            gnssStatus.verAcc = float(posInfo['verAcc'])
        if 'hdop' in posInfo and posInfo['hdop'] != None:
            gnssStatus.hdop = float(posInfo['hdop'])
        if 'vdop' in posInfo and posInfo['vdop'] != None:
            gnssStatus.vdop = float(posInfo['vdop'])
        if 'pdop' in posInfo and posInfo['pdop'] != None:
            gnssStatus.pdop = float(posInfo['pdop'])
        if 'origin_cs_ident' in posInfo and posInfo['origin_cs_ident'] != None:
            gnssStatus.cs = str(posInfo['origin_cs_ident'])
        if gnssStatus.cs != None and gnssStatus.cs != 'EPSG::4979':
            ok, gnssStatus.longitude, gnssStatus.latitude, gnssStatus.altitude = self._node._rieglVz.geosys.transformToWgs84(gnssStatus.cs, gnssStatus.longitude, gnssStatus.latitude, gnssStatus.altitude)
        return gnssStatus

    def _getGnssStatusFromEstimateInfo(self, estimInfo):
        gnssStatus = GnssStatus()
        #self._logger.debug("gnss (status): {}".format(estimInfo))
        if 'fix' in estimInfo and estimInfo['fix'] != None:
            gnssStatus.fix = estimInfo['fix']
        if 'num_sat' in estimInfo and estimInfo['num_sat'] != None:
            gnssStatus.numSat = estimInfo['num_sat']
        if 'longitude' in estimInfo and estimInfo['longitude'] != None:
            gnssStatus.longitude = float(estimInfo['longitude'])
        if 'latitude' in estimInfo and estimInfo['latitude'] != None:
            gnssStatus.latitude = float(estimInfo['latitude'])
        if 'height' in estimInfo and estimInfo['height'] != None:
            gnssStatus.altitude = float(estimInfo['height'])
        if 'hor_acc' in estimInfo and estimInfo['hor_acc'] != None:
            gnssStatus.horAcc = float(estimInfo['hor_acc'])
        if 'ver_acc' in estimInfo and estimInfo['ver_acc'] != None:
            gnssStatus.verAcc = float(estimInfo['ver_acc'])
        if 'hdop' in estimInfo and estimInfo['hdop'] != None:
            gnssStatus.hdop = float(estimInfo['hdop'])
        if 'vdop' in estimInfo and estimInfo['vdop'] != None:
            gnssStatus.vdop = float(estimInfo['vdop'])
        if 'pdop' in estimInfo and estimInfo['pdop'] != None:
            gnssStatus.pdop = float(estimInfo['pdop'])
        if 'recOriginCSIdent' in estimInfo and estimInfo['recOriginCSIdent'] != None:
            gnssStatus.cs = str(estimInfo['recOriginCSIdent'])
        if gnssStatus.cs != None and gnssStatus.cs != 'EPSG::4979':
            ok, gnssStatus.longitude, gnssStatus.latitude, gnssStatus.altitude = self._node._rieglVz.geosys.transformToWgs84(gnssStatus.cs, gnssStatus.longitude, gnssStatus.latitude, gnssStatus.altitude)
        return gnssStatus

    def _timer1sCallback(self):
        # gnss status
        gnssStatus = GnssStatus()
        try:
            if self._gnssSvc:
                estimInfo = json.loads(self._gnssSvc.estimateInfo())
                gnssStatus = self._getGnssStatusFromEstimateInfo(estimInfo)
            if self._gnssPosUpdateSigcon is None:
                gnssStatus.publish = False
            if self._scanSvc:
                gnssStatus.enabled = True if (self._scanSvc.gpsMode() != 0) else False
        except:
            gnssStatus.publish = False
            gnssStatus.err = True
        self.status.setGnssStatus(gnssStatus)

        # error status
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

        # camera status
        cameraStatus = CameraStatus()
        try:
            if self._camSvc:
                cameraList = self._camSvc.list()
                if len(cameraList) > 0:
                    cameraStatus.avail = True
        except:
            cameraStatus.err = True
        self.status.setCameraStatus(cameraStatus)

        # laser status
        laserOn = False
        try:
            if self._scanSvc:
                laserOn = self._scanSvc.isLaserOn()
        except:
            self.status._scannerStatus.err = True
        self.status.setLaserOn(laserOn)

    def shutdown(self):
        if not self._running:
            self._shutdownReq = True
            while self._shutdownReq:
                time.sleep(0.1)
