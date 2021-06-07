# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.11"

class ServiceSignalConnection(object):
    def __init__(self, signal, fn):
        self._signal = signal
        try:
            self._fn = weakref.WeakMethod(fn)
        except TypeError:
            self._fn = fn
        self._connected = True
    def disconnect(self):
        """Disconnect from the signal."""
        if self._connected:
            self._signal._disconnect(self)
            self._connected = False
    def connected(self):
        """Return true if the signal is still connected (disconnect() was not called)."""
        return self._connected

class ServiceSignal(object):
    def __init__(self, svc, signalName, decoderFn=None):
        self._svc = svc
        self._signalName = signalName
        self._connections = []
        self._lock = threading.Lock()
        self._decoderFn = decoderFn
    def connect(self, fn):
        """Connect the specified callback function to the signal."""
        sc = ServiceSignalConnection(self, fn)
        with self._lock:
            if len(self._connections) == 0:
                self._svc.subscribe(self._signalName, self._onSignalReceived)
            self._connections.append(sc)
        return sc
    def _disconnect(self, sc):
        with self._lock:
            if sc in self._connections:
                self._connections.remove(sc)
                sc._connected = False
                if len(self._connections) == 0:
                    self._svc.unsubscribe(self._signalName, self._onSignalReceived)
    def _onSignalReceived(self, payload):
        arg = self._decoderFn(payload) if self._decoderFn else None
        with self._lock:
            try:
                for sc in self._connections:
                    fn = sc._fn() if isinstance(sc._fn, weakref.WeakMethod) else sc._fn
                    if fn is not None:
                        if self._decoderFn is not None:
                            fn(arg)
                        else:
                            fn()
            except Exception as err:
                print(err)


class PoseEstimationSettings(dict):
    def __init__(self, *args, **kwargs):
        self['gnssAccuracy'] = None
        self['compassAccuracy'] = None
        self['forceGnss'] = None
        super().__init__(*args, **kwargs)
    @property
    def gnssAccuracy(self):
        return self['gnssAccuracy']
    @gnssAccuracy.setter
    def gnssAccuracy(self, value):
        self['gnssAccuracy'] = value
    @property
    def compassAccuracy(self):
        return self['compassAccuracy']
    @compassAccuracy.setter
    def compassAccuracy(self, value):
        self['compassAccuracy'] = value
    @property
    def forceGnss(self):
        return self['forceGnss']
    @forceGnss.setter
    def forceGnss(self, value):
        self['forceGnss'] = value

class ReflectorSearchSettings(dict):
    def __init__(self, *args, **kwargs):
        self['minDiameter'] = None
        self['maxDiameter'] = None
        self['minRange'] = None
        self['maxRange'] = None
        self['minReflectance'] = None
        self['maxReflectors'] = None
        self['mode'] = None
        self['models'] = []
        self['resolveMta'] = None
        super().__init__(*args, **kwargs)
    @property
    def minDiameter(self):
        return self['minDiameter']
    @minDiameter.setter
    def minDiameter(self, value):
        self['minDiameter'] = value
    @property
    def maxDiameter(self):
        return self['maxDiameter']
    @maxDiameter.setter
    def maxDiameter(self, value):
        self['maxDiameter'] = value
    @property
    def minRange(self):
        return self['minRange']
    @minRange.setter
    def minRange(self, value):
        self['minRange'] = value
    @property
    def maxRange(self):
        return self['maxRange']
    @maxRange.setter
    def maxRange(self, value):
        self['maxRange'] = value
    @property
    def minReflectance(self):
        return self['minReflectance']
    @minReflectance.setter
    def minReflectance(self, value):
        self['minReflectance'] = value
    @property
    def maxReflectors(self):
        return self['maxReflectors']
    @maxReflectors.setter
    def maxReflectors(self, value):
        self['maxReflectors'] = value
    @property
    def mode(self):
        return self['mode']
    @mode.setter
    def mode(self, value):
        self['mode'] = value
    @property
    def models(self):
        return self['models']
    @models.setter
    def models(self, value):
        self['models'] = value
    @property
    def resolveMta(self):
        return self['resolveMta']
    @resolveMta.setter
    def resolveMta(self, value):
        self['resolveMta'] = value

class ReflectorScanSettings(dict):
    def __init__(self, *args, **kwargs):
        self['overlap'] = None
        self['oversize'] = None
        super().__init__(*args, **kwargs)
    @property
    def overlap(self):
        return self['overlap']
    @overlap.setter
    def overlap(self, value):
        self['overlap'] = value
    @property
    def oversize(self):
        return self['oversize']
    @oversize.setter
    def oversize(self, value):
        self['oversize'] = value

class ScanSequenceInformation(dict):
    def __init__(self, *args, **kwargs):
        self['success'] = None
        self['canceled'] = None
        self['errorMessage'] = None
        self['media'] = None
        self['project'] = None
        self['scanposition'] = None
        self['resultJson'] = None
        self['filePrefix'] = None
        super().__init__(*args, **kwargs)
    @property
    def success(self):
        return self['success']
    @success.setter
    def success(self, value):
        self['success'] = value
    @property
    def canceled(self):
        return self['canceled']
    @canceled.setter
    def canceled(self, value):
        self['canceled'] = value
    @property
    def errorMessage(self):
        return self['errorMessage']
    @errorMessage.setter
    def errorMessage(self, value):
        self['errorMessage'] = value
    @property
    def media(self):
        return self['media']
    @media.setter
    def media(self, value):
        self['media'] = value
    @property
    def project(self):
        return self['project']
    @project.setter
    def project(self, value):
        self['project'] = value
    @property
    def scanposition(self):
        return self['scanposition']
    @scanposition.setter
    def scanposition(self, value):
        self['scanposition'] = value
    @property
    def resultJson(self):
        return self['resultJson']
    @resultJson.setter
    def resultJson(self, value):
        self['resultJson'] = value
    @property
    def filePrefix(self):
        return self['filePrefix']
    @filePrefix.setter
    def filePrefix(self, value):
        self['filePrefix'] = value

class AcquisitionInformation(dict):
    def __init__(self, *args, **kwargs):
        self['success'] = None
        self['canceled'] = None
        self['errorMessage'] = None
        self['media'] = None
        self['project'] = None
        self['scanposition'] = None
        self['resultJson'] = None
        self['filePrefix'] = None
        super().__init__(*args, **kwargs)
    @property
    def success(self):
        return self['success']
    @success.setter
    def success(self, value):
        self['success'] = value
    @property
    def canceled(self):
        return self['canceled']
    @canceled.setter
    def canceled(self, value):
        self['canceled'] = value
    @property
    def errorMessage(self):
        return self['errorMessage']
    @errorMessage.setter
    def errorMessage(self, value):
        self['errorMessage'] = value
    @property
    def media(self):
        return self['media']
    @media.setter
    def media(self, value):
        self['media'] = value
    @property
    def project(self):
        return self['project']
    @project.setter
    def project(self, value):
        self['project'] = value
    @property
    def scanposition(self):
        return self['scanposition']
    @scanposition.setter
    def scanposition(self, value):
        self['scanposition'] = value
    @property
    def resultJson(self):
        return self['resultJson']
    @resultJson.setter
    def resultJson(self, value):
        self['resultJson'] = value
    @property
    def filePrefix(self):
        return self['filePrefix']
    @filePrefix.setter
    def filePrefix(self, value):
        self['filePrefix'] = value

class TaskModeChangedPayload(dict):
    def __init__(self, *args, **kwargs):
        self['task'] = None
        self['oldMode'] = None
        self['newMode'] = None
        super().__init__(*args, **kwargs)
    @property
    def task(self):
        return self['task']
    @task.setter
    def task(self, value):
        self['task'] = value
    @property
    def oldMode(self):
        return self['oldMode']
    @oldMode.setter
    def oldMode(self, value):
        self['oldMode'] = value
    @property
    def newMode(self):
        return self['newMode']
    @newMode.setter
    def newMode(self, value):
        self['newMode'] = value

def _controlservice_error_decoder(payload):
    return json.loads(payload.decode())
def _controlservice_horizontalimageoverlapchanged_decoder(payload):
    return struct.unpack("!B", payload)[0]
def _controlservice_imagecapturemodechanged_decoder(payload):
    return struct.unpack("!i", payload)[0]
def _controlservice_taskmodechanged_decoder(payload):
    return TaskModeChangedPayload(json.loads(payload.decode()))
def _controlservice_storemeasurementstreamchanged_decoder(payload):
    return struct.unpack("!?", payload)[0]
def _controlservice_storemonitorstreamchanged_decoder(payload):
    return struct.unpack("!?", payload)[0]
def _controlservice_activeusbstoragedevicechanged_decoder(payload):
    return payload.decode()
def _controlservice_activesdcstoragedevicechanged_decoder(payload):
    return payload.decode()
def _controlservice_reflectorsearchsettingschanged_decoder(payload):
    return ReflectorSearchSettings(json.loads(payload.decode()))
def _controlservice_reflectorscansettingschanged_decoder(payload):
    return ReflectorScanSettings(json.loads(payload.decode()))
def _controlservice_poseestimationsettingschanged_decoder(payload):
    return PoseEstimationSettings(json.loads(payload.decode()))
def _controlservice_registrationmodechanged_decoder(payload):
    return struct.unpack("!i", payload)[0]
def _controlservice_registrationresolvemtachanged_decoder(payload):
    return struct.unpack("!?", payload)[0]
def _controlservice_environmentsensorsenabledchanged_decoder(payload):
    return struct.unpack("!?", payload)[0]
def _controlservice_registrationbranchpointchanged_decoder(payload):
    return payload.decode()
def _controlservice_acquisitionstarted_decoder(payload):
    return struct.unpack("!i", payload)[0]
def _controlservice_acquisitionfinished_decoder(payload):
    return struct.unpack("!i", payload)[0]
def _controlservice_taskstarted_decoder(payload):
    return payload.decode()
def _controlservice_taskprogress_decoder(payload):
    return payload.decode()
def _controlservice_taskfinished_decoder(payload):
    return payload.decode()
def _controlservice_taskstatechanged_decoder(payload):
    return payload.decode()
def _controlservice_backgroundtaskadded_decoder(payload):
    return payload.decode()
def _controlservice_backgroundtaskchanged_decoder(payload):
    return payload.decode()
def _controlservice_backgroundtaskremoved_decoder(payload):
    return payload.decode()

class ControlService(object):
    """The control service.
       
       The control service implements the logic of the data acquisition. It can be
       used to execute a scan sequence consisting of different data acquisition
       tasks like a pose estimation, scan data acquisition, reflector search,
       reflector scans, and image acquisition. The control service does not
       implement all of the necessary functionality itself but instead uses other
       services like the PoseEstimationService or DataProcService to perform the
       individual data acquisition tasks.
       
       Acquisition tasks can be in one of three modes (ON, OFF, AUTO_ON). When a task
       is in mode ON then it will be executed or return with an error it can't be
       executed. If a task is in OFF mode then it will not be executed.
       
       The AUTO_ON mode is slightly different for each task:
           pose estimation:   executed if no pose estimation has been done for the
                              active scanposition
           reflector search:  executed if scan task is part of scan sequence
           reflector scans:   executed if reflector search is part of scan sequence
           image acquisition: executed if camera is detected and camera calibration
                              is set
       
       Changelog v1.4:
       ---------------
       The modes IMAGE_FAST and IMAGE_FAST_AUTO_ON are experimental and can be used to
       perform the image acquisition at the same time as the scan acquisition. The camera
       exposure time gets clamped if necessary to ensure that the images can be triggered
       at the right moment and also to minimize motion blur."""

    class AcquisitionMode(enum.IntEnum):
        ACM_SCANSEQUENCE = 1
        ACM_QUICKSCAN = 2

    class AcquisitionTask(enum.IntEnum):
        ACT_SCAN = 1
        ACT_IMAGESEQUENCE = 2
        ACT_POSEESTIMATION = 3
        ACT_REFLSEARCH = 4
        ACT_REFLSCANS = 5
        ACT_FINALIZATION = 255

    class AcquisitionTaskMode(enum.IntEnum):
        ACTM_AUTO_ON = 1
        ACTM_ON = 2
        ACTM_OFF = 3
        ACTM_IMAGE_FAST = 20
        ACTM_IMAGE_FAST_AUTO_ON = 21

    class AcquisitionTaskState(enum.IntEnum):
        ACTS_NOT_RUNNING = 0
        ACTS_RUNNING = 1
        ACTS_PAUSED = 2

    class ResultCode(enum.IntEnum):
        RC_SUCCESS = 0
        RC_CANCELED = 1
        RC_ERROR = 2

    class ImageAcquisitionCaptureMode(enum.IntEnum):
        ICM_AUTOMATIC = 0
        ICM_INTERACTIVE = 1

    class AcquisitionFlags(enum.IntEnum):
        ACQ_SCAN = 1
        ACQ_POSEESTIMATION_BEFORE_SCAN = 2
        ACQ_REFLSEARCH = 4
        ACQ_REFLSCANS = 8
        ACQ_IMAGES_AFTER_SCAN = 16
        ACQ_IMAGES_DURING_SCAN = 32
        ACQ_FINALIZATION_IMU_DELAY = 64

    class RegistrationMode(enum.IntEnum):
        REG_DISABLED = 0
        REG_AUTO = 1
        REG_OUTDOOR_URBAN = 2
        REG_OUTDOOR_NON_URBAN = 3
        REG_INDOOR_SMALL = 4
        REG_INDOOR_LARGE = 5
        REG_MINING_MEDIUM = 6
        REG_MINING_LARGE = 7

    ACM_SCANSEQUENCE = AcquisitionMode.ACM_SCANSEQUENCE
    ACM_QUICKSCAN = AcquisitionMode.ACM_QUICKSCAN
    ACT_SCAN = AcquisitionTask.ACT_SCAN
    ACT_IMAGESEQUENCE = AcquisitionTask.ACT_IMAGESEQUENCE
    ACT_POSEESTIMATION = AcquisitionTask.ACT_POSEESTIMATION
    ACT_REFLSEARCH = AcquisitionTask.ACT_REFLSEARCH
    ACT_REFLSCANS = AcquisitionTask.ACT_REFLSCANS
    ACT_FINALIZATION = AcquisitionTask.ACT_FINALIZATION
    ACTM_AUTO_ON = AcquisitionTaskMode.ACTM_AUTO_ON
    ACTM_ON = AcquisitionTaskMode.ACTM_ON
    ACTM_OFF = AcquisitionTaskMode.ACTM_OFF
    ACTM_IMAGE_FAST = AcquisitionTaskMode.ACTM_IMAGE_FAST
    ACTM_IMAGE_FAST_AUTO_ON = AcquisitionTaskMode.ACTM_IMAGE_FAST_AUTO_ON
    ACTS_NOT_RUNNING = AcquisitionTaskState.ACTS_NOT_RUNNING
    ACTS_RUNNING = AcquisitionTaskState.ACTS_RUNNING
    ACTS_PAUSED = AcquisitionTaskState.ACTS_PAUSED
    RC_SUCCESS = ResultCode.RC_SUCCESS
    RC_CANCELED = ResultCode.RC_CANCELED
    RC_ERROR = ResultCode.RC_ERROR
    ICM_AUTOMATIC = ImageAcquisitionCaptureMode.ICM_AUTOMATIC
    ICM_INTERACTIVE = ImageAcquisitionCaptureMode.ICM_INTERACTIVE
    ACQ_SCAN = AcquisitionFlags.ACQ_SCAN
    ACQ_POSEESTIMATION_BEFORE_SCAN = AcquisitionFlags.ACQ_POSEESTIMATION_BEFORE_SCAN
    ACQ_REFLSEARCH = AcquisitionFlags.ACQ_REFLSEARCH
    ACQ_REFLSCANS = AcquisitionFlags.ACQ_REFLSCANS
    ACQ_IMAGES_AFTER_SCAN = AcquisitionFlags.ACQ_IMAGES_AFTER_SCAN
    ACQ_IMAGES_DURING_SCAN = AcquisitionFlags.ACQ_IMAGES_DURING_SCAN
    ACQ_FINALIZATION_IMU_DELAY = AcquisitionFlags.ACQ_FINALIZATION_IMU_DELAY
    REG_DISABLED = RegistrationMode.REG_DISABLED
    REG_AUTO = RegistrationMode.REG_AUTO
    REG_OUTDOOR_URBAN = RegistrationMode.REG_OUTDOOR_URBAN
    REG_OUTDOOR_NON_URBAN = RegistrationMode.REG_OUTDOOR_NON_URBAN
    REG_INDOOR_SMALL = RegistrationMode.REG_INDOOR_SMALL
    REG_INDOOR_LARGE = RegistrationMode.REG_INDOOR_LARGE
    REG_MINING_MEDIUM = RegistrationMode.REG_MINING_MEDIUM
    REG_MINING_LARGE = RegistrationMode.REG_MINING_LARGE

    def __init__(self, address):
        self._svc = riconnect.Service("ControlService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_controlservice_error_decoder)
        self._horizontalImageOverlapChanged = ServiceSignal(self._svc, "horizontalImageOverlapChanged", decoderFn=_controlservice_horizontalimageoverlapchanged_decoder)
        self._imageCaptureModeChanged = ServiceSignal(self._svc, "imageCaptureModeChanged", decoderFn=_controlservice_imagecapturemodechanged_decoder)
        self._taskModeChanged = ServiceSignal(self._svc, "taskModeChanged", decoderFn=_controlservice_taskmodechanged_decoder)
        self._storeMeasurementStreamChanged = ServiceSignal(self._svc, "storeMeasurementStreamChanged", decoderFn=_controlservice_storemeasurementstreamchanged_decoder)
        self._storeMonitorStreamChanged = ServiceSignal(self._svc, "storeMonitorStreamChanged", decoderFn=_controlservice_storemonitorstreamchanged_decoder)
        self._activeUsbStorageDeviceChanged = ServiceSignal(self._svc, "activeUsbStorageDeviceChanged", decoderFn=_controlservice_activeusbstoragedevicechanged_decoder)
        self._activeSdcStorageDeviceChanged = ServiceSignal(self._svc, "activeSdcStorageDeviceChanged", decoderFn=_controlservice_activesdcstoragedevicechanged_decoder)
        self._reflectorSearchSettingsChanged = ServiceSignal(self._svc, "reflectorSearchSettingsChanged", decoderFn=_controlservice_reflectorsearchsettingschanged_decoder)
        self._reflectorScanSettingsChanged = ServiceSignal(self._svc, "reflectorScanSettingsChanged", decoderFn=_controlservice_reflectorscansettingschanged_decoder)
        self._poseEstimationSettingsChanged = ServiceSignal(self._svc, "poseEstimationSettingsChanged", decoderFn=_controlservice_poseestimationsettingschanged_decoder)
        self._registrationModeChanged = ServiceSignal(self._svc, "registrationModeChanged", decoderFn=_controlservice_registrationmodechanged_decoder)
        self._registrationResolveMtaChanged = ServiceSignal(self._svc, "registrationResolveMtaChanged", decoderFn=_controlservice_registrationresolvemtachanged_decoder)
        self._environmentSensorsEnabledChanged = ServiceSignal(self._svc, "environmentSensorsEnabledChanged", decoderFn=_controlservice_environmentsensorsenabledchanged_decoder)
        self._registrationBranchPointChanged = ServiceSignal(self._svc, "registrationBranchPointChanged", decoderFn=_controlservice_registrationbranchpointchanged_decoder)
        self._acquisitionStarted = ServiceSignal(self._svc, "acquisitionStarted", decoderFn=_controlservice_acquisitionstarted_decoder)
        self._acquisitionFinished = ServiceSignal(self._svc, "acquisitionFinished", decoderFn=_controlservice_acquisitionfinished_decoder)
        self._taskStarted = ServiceSignal(self._svc, "taskStarted", decoderFn=_controlservice_taskstarted_decoder)
        self._taskProgress = ServiceSignal(self._svc, "taskProgress", decoderFn=_controlservice_taskprogress_decoder)
        self._taskFinished = ServiceSignal(self._svc, "taskFinished", decoderFn=_controlservice_taskfinished_decoder)
        self._taskStateChanged = ServiceSignal(self._svc, "taskStateChanged", decoderFn=_controlservice_taskstatechanged_decoder)
        self._backgroundTaskAdded = ServiceSignal(self._svc, "backgroundTaskAdded", decoderFn=_controlservice_backgroundtaskadded_decoder)
        self._backgroundTaskChanged = ServiceSignal(self._svc, "backgroundTaskChanged", decoderFn=_controlservice_backgroundtaskchanged_decoder)
        self._backgroundTaskRemoved = ServiceSignal(self._svc, "backgroundTaskRemoved", decoderFn=_controlservice_backgroundtaskremoved_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def horizontalImageOverlap(self):
        """Return horizontal image overlap in percent used during image acquisition.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("horizontalImageOverlap_8587ce50", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setHorizontalImageOverlap(self, value):
        """Set horizontal image overlap used during image acquisition (in percent).
           
           Arguments:
             value (int): the new horizontal image overlap"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setHorizontalImageOverlap_01b541b0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def imageCaptureMode(self):
        """Return the image capture mode used during image acquisition.
           
           Returns: ImageAcquisitionCaptureMode"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imageCaptureMode_9344dcc5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ControlService.ImageAcquisitionCaptureMode(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setImageCaptureMode(self, mode):
        """Set the image capture mode used during image acquisition.
           
           Arguments:
             mode (ImageAcquisitionCaptureMode): the new capture mode"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(mode)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setImageCaptureMode_1880b858", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def setTaskMode(self, task, mode):
        """Set a task's execution mode.
           
           Arguments:
             task (AcquisitionTask): the acquisition task
             mode (AcquisitionTaskMode): the new task execution mode"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = int(task)
        inputs[1].i32 = int(mode)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setTaskMode_ff9d8b67", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def taskMode(self, task):
        """Return the execution mode of a given task.
           
           Arguments:
             task (AcquisitionTask): the acquisition task
           
           Returns: AcquisitionTaskMode"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(task)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("taskMode_880c8f3c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ControlService.AcquisitionTaskMode(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def monitorStreamUri(self):
        """Return the monitor stream URI.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("monitorStreamUri_99c9f421", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def maxPreviewWidth(self):
        """Return the maximum width of the scan preview image (in pixels).
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("maxPreviewWidth_8eea7fc4", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMaxPreviewWidth(self, value):
        """Set the maximum width of the scan preview image (in pixels).
           
           If the horizontal field of view (FOV) of the scan is bigger than the vertical
           FOV then this value is used as preview width. The preview height is calculated
           using the aspect ratio of the FOV.
           
           Arguments:
             value (int): the new maximum preview image width"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMaxPreviewWidth_043a8433", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def maxPreviewHeight(self):
        """Return the maximum height of the scan preview image (in pixels).
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("maxPreviewHeight_a099ce70", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMaxPreviewHeight(self, value):
        """Set the maximum height of the scan preview image (in pixels).
           
           If the vertical field of view (FOV) of the scan is bigger than the horizontal
           FOV then this value is used as preview height. The preview width is calculated
           using the aspect ratio of the FOV.
           
           Arguments:
             value (int): the new maximum preview image height"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMaxPreviewHeight_4780c193", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def storeMeasurementStream(self):
        """Return true if the scan measurement stream should be stored on the storage media, otherwise return false.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("storeMeasurementStream_1e5e42f9", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setStoreMeasurementStream(self, value):
        """Set whether the scan measurement stream should or should not be stored on the storage media.
           
           Arguments:
             value (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if value else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setStoreMeasurementStream_4408c4c0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def storeMonitorStream(self):
        """Return true if the scan monitor stream should be stored on the storage media, otherwise return false.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("storeMonitorStream_3b2a8e81", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setStoreMonitorStream(self, value):
        """Set whether the scan monitor stream should or should not be stored on the storage media.
           
           Arguments:
             value (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if value else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setStoreMonitorStream_76fbaa8a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def scanpatternDirFactory(self):
        """Return the path of the directory where the factory scan patterns are located.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanpatternDirFactory_fff019f9", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def scanpatternDirUser(self):
        """Return the path of the directory where the user defined scan patterns are located.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanpatternDirUser_bc3a97a1", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def startScanSequence(self, *args):
        """
           startScanSequence()
               Start data acquisition for the active project and scanposition.
               
               This function is deprecated. Use startAcquisition() instead.

           startScanSequence(doPoseEstimation, doScan, doReflectorSearch, doReflectorScans, doImageSequence)
               Start data acquisition for the active project and scanposition.
               
               This function is deprecated. Use startAcquisition(...) instead.
               
               Arguments:
                 doPoseEstimation (bool): 
                 doScan (bool): 
                 doReflectorSearch (bool): 
                 doReflectorScans (bool): 
                 doImageSequence (bool): 

        """
        if len(args) == 0:
            self._startScanSequence_33a8780a(*args)
        elif len(args) == 5:
            self._startScanSequence_2b2b6b7c(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _startScanSequence_33a8780a(self):
        """Start data acquisition for the active project and scanposition.
           
           This function is deprecated. Use startAcquisition() instead."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startScanSequence_33a8780a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def _startScanSequence_2b2b6b7c(self, doPoseEstimation, doScan, doReflectorSearch, doReflectorScans, doImageSequence):
        """Start data acquisition for the active project and scanposition.
           
           This function is deprecated. Use startAcquisition(...) instead.
           
           Arguments:
             doPoseEstimation (bool): 
             doScan (bool): 
             doReflectorSearch (bool): 
             doReflectorScans (bool): 
             doImageSequence (bool): """
        inputs = [riconnect.Value() for i in range(0, 5)]
        inputs[0].i32 = 1 if doPoseEstimation else 0
        inputs[1].i32 = 1 if doScan else 0
        inputs[2].i32 = 1 if doReflectorSearch else 0
        inputs[3].i32 = 1 if doReflectorScans else 0
        inputs[4].i32 = 1 if doImageSequence else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startScanSequence_2b2b6b7c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def startAcquisition(self, *args):
        """
           startAcquisition()
               Start data acquisition for the active project and scanposition.
               
               Starts the execution of a sequence of data acquisition tasks and stores
               the results in the directory of the active scan position. The number of
               tasks executed depends on the currently set task execution modes.
               
               New in version 1.1.

           startAcquisition(doPoseEstimation, doScan, doReflectorSearch, doReflectorScans, doImageSequence)
               Start data acquisition for the active project and scanposition.
               
               This method executes the data acquisition with the specified tasks without
               modifying the currently set task execution modes.
               
               Please note that starting an acquisition with this function does not
               support the image acquisition during scan.
               
               New in version 1.1.
               
               Arguments:
                 doPoseEstimation (bool): 
                 doScan (bool): 
                 doReflectorSearch (bool): 
                 doReflectorScans (bool): 
                 doImageSequence (bool): 

           startAcquisition(flags)
               Start data acquisition for the active project and scanposition.
               
               This method executes the data acquisition with the specified tasks without
               modifying the currently set task execution modes.
               
               New in version 1.5.
               
               Arguments:
                 flags (int): The tasks to execute (see AcquisitionFlags).

        """
        if len(args) == 0:
            self._startAcquisition_f6362f4d(*args)
        elif len(args) == 5:
            self._startAcquisition_511a7f91(*args)
        elif len(args) == 1:
            self._startAcquisition_2274d482(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _startAcquisition_f6362f4d(self):
        """Start data acquisition for the active project and scanposition.
           
           Starts the execution of a sequence of data acquisition tasks and stores
           the results in the directory of the active scan position. The number of
           tasks executed depends on the currently set task execution modes.
           
           New in version 1.1."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startAcquisition_f6362f4d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def _startAcquisition_511a7f91(self, doPoseEstimation, doScan, doReflectorSearch, doReflectorScans, doImageSequence):
        """Start data acquisition for the active project and scanposition.
           
           This method executes the data acquisition with the specified tasks without
           modifying the currently set task execution modes.
           
           Please note that starting an acquisition with this function does not
           support the image acquisition during scan.
           
           New in version 1.1.
           
           Arguments:
             doPoseEstimation (bool): 
             doScan (bool): 
             doReflectorSearch (bool): 
             doReflectorScans (bool): 
             doImageSequence (bool): """
        inputs = [riconnect.Value() for i in range(0, 5)]
        inputs[0].i32 = 1 if doPoseEstimation else 0
        inputs[1].i32 = 1 if doScan else 0
        inputs[2].i32 = 1 if doReflectorSearch else 0
        inputs[3].i32 = 1 if doReflectorScans else 0
        inputs[4].i32 = 1 if doImageSequence else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startAcquisition_511a7f91", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def _startAcquisition_2274d482(self, flags):
        """Start data acquisition for the active project and scanposition.
           
           This method executes the data acquisition with the specified tasks without
           modifying the currently set task execution modes.
           
           New in version 1.5.
           
           Arguments:
             flags (int): The tasks to execute (see AcquisitionFlags)."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = flags
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startAcquisition_2274d482", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def startQuickscan(self, *args):
        """
           startQuickscan()
               Start a scan that will be stored in the quickscan directory.

           startQuickscan(targetDir, basename)
               Start a scan that will be stored in the specified directory using the specified file base name.
               
               New in version 1.3.
               
               Arguments:
                 targetDir (str): absolute path to target directory
                 basename (str): base name of files (e.g. myscan_01)

        """
        if len(args) == 0:
            self._startQuickscan_05c74ca9(*args)
        elif len(args) == 2:
            self._startQuickscan_f6ca40e0(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _startQuickscan_05c74ca9(self):
        """Start a scan that will be stored in the quickscan directory."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startQuickscan_05c74ca9", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def _startQuickscan_f6ca40e0(self, targetDir, basename):
        """Start a scan that will be stored in the specified directory using the specified file base name.
           
           New in version 1.3.
           
           Arguments:
             targetDir (str): absolute path to target directory
             basename (str): base name of files (e.g. myscan_01)"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = targetDir
        inputs[1].s = basename
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startQuickscan_f6ca40e0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def stop(self):
        """Stop the running data acquisition."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("stop_ef399b2d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def lastScanSequence(self):
        """Return information about the last executed scan sequence.
           
           This function is deprecated. Use lastAcquisition() instead.
           
           Returns: ScanSequenceInformation"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("lastScanSequence_7ae59e14", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ScanSequenceInformation(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def lastAcquisition(self):
        """Return information about the last executed data acquisition.
           
           New in version 1.1.
           
           Returns: AcquisitionInformation"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("lastAcquisition_d82874a9", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(AcquisitionInformation(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def pauseTask(self, task):
        """Pause a running acquisition task.
           
           Returns false if the specified task is not running or can't be paused.
           Please note that the task might not be paused immediately. The signal
           taskStateChanged() will be emitted when the task's state changed.
           
           Arguments:
             task (AcquisitionTask): the acquisition task to pause
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(task)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("pauseTask_b3536795", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def resumeTask(self, task):
        """Resume a paused acquisition task.
           
           Returns false if the specified task is not running; otherwise returns true.
           
           Arguments:
             task (AcquisitionTask): the paused acquisition task
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(task)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("resumeTask_6355a317", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def restartTask(self, task):
        """Restart a running or paused acquisition task.
           
           Returns false if the specified task is not running or can't be restarted,
           otherwise returns true.
           
           Arguments:
             task (AcquisitionTask): the acquisition task to restart
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(task)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("restartTask_17a059a5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def isBusy(self):
        """Return true if any data acquisition task is running.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isBusy_3fa264f2", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def loadScanPattern(self, pattern):
        """Load a scan pattern.
           
           Arguments:
             pattern (str): the name of the pattern"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = pattern
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("loadScanPattern_39d52fa1", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def saveScanPattern(self, pattern):
        """Save a scan pattern.
           
           If a pattern with the given name does already exist and is a user created
           pattern then it will be replaced.
           
           Arguments:
             pattern (str): the name of the pattern"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = pattern
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("saveScanPattern_3678c97a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def scanPatterns(self):
        """Get list of available scan patterns.
           
           Returns: list(str)
             the available scan patterns"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanPatterns_3fa701bf", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        _ot_patterns = []
        _otsplits = rtransfers[0][0:-1].split(b"\0") if rtransfers[0] else []
        for part in _otsplits:
            _ot_patterns.append(part.decode())
        outputs.append(_ot_patterns)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def scanPatternsDetailed(self):
        """Return detailed information about the available scan patterns.
           
           The data is returned as JSON string in the following format:
           [
             {
               "name": "Panorama_50",
               "mode": "rectFov",
               "phiIncrement": 0.05,
               "phiStart": 0,
               "phiStop": 360,
               "thetaIncrement": 0.05,
               "thetaStart": 30,
               "thetaStop": 130
             },
             ...
           ]
           
           New in version 1.3.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanPatternsDetailed_50f86490", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def deleteScanPattern(self, pattern):
        """Delete a scan pattern.
           
           Returns true if the scan pattern was successfully deleted.
           Please note that only user defined scan patterns can be deleted.
           
           New in version 1.5.
           
           Arguments:
             pattern (str): the name of the pattern
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = pattern
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("deleteScanPattern_1b142f83", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def acquisitionMode(self):
        """Return the current data acquisition mode.
           
           Returns: AcquisitionMode"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("acquisitionMode_16d8b153", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ControlService.AcquisitionMode(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def activeUsbStorageDevice(self):
        """Return the name of the active USB storage device.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("activeUsbStorageDevice_d9260b57", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setActiveUsbStorageDevice(self, name):
        """Set the active USB storage device.
           
           Please refer to the InterfaceService to get a list of the
           available USB devices.
           
           Arguments:
             name (str): the name of the device"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setActiveUsbStorageDevice_4de43c16", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def activeSdcStorageDevice(self):
        """Return the name of the active SD-Card storage device.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("activeSdcStorageDevice_5c85cca6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setActiveSdcStorageDevice(self, name):
        """Set the active SD-Card storage device.
           
           Please refer to the InterfaceService to get a list of the
           available SD-Card devices.
           
           Arguments:
             name (str): the name of device"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setActiveSdcStorageDevice_196e1a39", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def reflectorSearchSettings(self):
        """Return the parameters used during reflector search.
           
           Returns: ReflectorSearchSettings"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("reflectorSearchSettings_00c081a3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ReflectorSearchSettings(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setReflectorSearchSettings(self, settings):
        """Set the parameters used during reflector search.
           
           The simple reflector search uses the search settings:
             - minDiameter
             - maxDiameter
             - minRange
             - maxRange
             - minReflectance
             - maxReflectors
           
           The model based reflector search (since version 1.8) uses the search settings:
             - models
           
           Use supportedReflectorSearchModels() to get a list of supported search models.
           
           Arguments:
             settings (ReflectorSearchSettings): the new settings"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = json.dumps(settings)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setReflectorSearchSettings_aed3e188", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def reflectorScanSettings(self):
        """Return the parameters used during reflector scans.
           
           Returns: ReflectorScanSettings"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("reflectorScanSettings_fce6a68f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ReflectorScanSettings(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setReflectorScanSettings(self, settings):
        """Set the parameters used during reflector scans.
           
           Arguments:
             settings (ReflectorScanSettings): the new settings"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = json.dumps(settings)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setReflectorScanSettings_5a6cd0d0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def poseEstimationSettings(self):
        """Return the parameters used during pose estimation.
           
           Returns: PoseEstimationSettings"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("poseEstimationSettings_a003c79a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(PoseEstimationSettings(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setPoseEstimationSettings(self, settings):
        """Set the parameters used during pose estimation.
           
           Arguments:
             settings (PoseEstimationSettings): the new settings"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = json.dumps(settings)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setPoseEstimationSettings_71b4a5ea", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def backgroundTasks(self):
        """Return information about all queued background tasks.
           
           New in version 1.1.
           
           Returns: str
             JSON array"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("backgroundTasks_ea7057eb", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def backgroundTask(self, taskId):
        """Return information about a background task.
           
           New in version 1.1.
           
           Arguments:
             taskId (int): ID of background task
           
           Returns: str
             JSON object"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = taskId
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("backgroundTask_e5ee2edd", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def cancelBackgroundTask(self, taskId):
        """Cancel background task.
           
           New in version 1.1.
           
           Arguments:
             taskId (int): ID of background task"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = taskId
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("cancelBackgroundTask_712396dd", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def addRdbCreationTask(self, *args):
        """
           addRdbCreationTask(storageMedia, project, scanposition, scan)
               Add a background task to create a point cloud (RDB) from raw scan data (RXP).
               
               During RDB creation the raw RXP data is read and each echo is moved to the most
               likely MTA zone before it is added to the point cloud.
               
               New in version 1.1.
               
               Arguments:
                 storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
                 project (str): project name
                 scanposition (str): name of scan position
                 scan (str): name of scan
               
               Returns: int
                 ID of background task

           addRdbCreationTask(storageMedia, project, scanposition, scan, resolveMta)
               Add a background task to create a point cloud (RDB) from raw scan data (RXP).
               
               Overloaded method that allows to disable MTA resolution of scan.
               
               Creates point cloud from scan data.
               
               New in version 1.9.
               
               Arguments:
                 storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
                 project (str): project name
                 scanposition (str): name of scan position
                 scan (str): name of scan
                 resolveMta (bool): false to disable MTA resolution of scan
               
               Returns: int
                 ID of background task

        """
        if len(args) == 4:
            return self._addRdbCreationTask_78ebb721(*args)
        elif len(args) == 5:
            return self._addRdbCreationTask_8ce055e4(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _addRdbCreationTask_78ebb721(self, storageMedia, project, scanposition, scan):
        """Add a background task to create a point cloud (RDB) from raw scan data (RXP).
           
           During RDB creation the raw RXP data is read and each echo is moved to the most
           likely MTA zone before it is added to the point cloud.
           
           New in version 1.1.
           
           Arguments:
             storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
             project (str): project name
             scanposition (str): name of scan position
             scan (str): name of scan
           
           Returns: int
             ID of background task"""
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].s = storageMedia
        inputs[1].s = project
        inputs[2].s = scanposition
        inputs[3].s = scan
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("addRdbCreationTask_78ebb721", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _addRdbCreationTask_8ce055e4(self, storageMedia, project, scanposition, scan, resolveMta):
        """Add a background task to create a point cloud (RDB) from raw scan data (RXP).
           
           Overloaded method that allows to disable MTA resolution of scan.
           
           Creates point cloud from scan data.
           
           New in version 1.9.
           
           Arguments:
             storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
             project (str): project name
             scanposition (str): name of scan position
             scan (str): name of scan
             resolveMta (bool): false to disable MTA resolution of scan
           
           Returns: int
             ID of background task"""
        inputs = [riconnect.Value() for i in range(0, 5)]
        inputs[0].s = storageMedia
        inputs[1].s = project
        inputs[2].s = scanposition
        inputs[3].s = scan
        inputs[4].i32 = 1 if resolveMta else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("addRdbCreationTask_8ce055e4", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def addRegistrationTask(self, storageMedia, project, scanposition, mode):
        """Add a background task to register a scan position.
           
           If the specified project does already have a registered scan position
           then the mode of the already registered scan position is used instead
           of the specified one.
           
           Creating a new registration task does also create an RDB creation task
           if necessary.
           
           New in version 1.1.
           
           Arguments:
             storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
             project (str): project name
             scanposition (str): name of scan position
             mode (RegistrationMode): the registration mode
           
           Returns: int
             ID of background task"""
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].s = storageMedia
        inputs[1].s = project
        inputs[2].s = scanposition
        inputs[3].i32 = int(mode)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("addRegistrationTask_8bd9f793", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def registrationMode(self):
        """Return point cloud registration mode used for automatic registration.
           
           New in version 1.1.
           
           Returns: RegistrationMode"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("registrationMode_d38f593e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ControlService.RegistrationMode(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setRegistrationMode(self, mode):
        """Set point cloud registration mode used for automatic registration.
           
           If the set registration mode is anything other than REG_DISABLED then
           the first acquired scan of a scan position will be autoamatically queued
           for registration after the acquisition of the scan finished successfully.
           
           New in version 1.1.
           
           Arguments:
             mode (RegistrationMode): new registration mode"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(mode)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setRegistrationMode_4f1f5ddd", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def registrationResolveMta(self):
        """Return true if MTA resolution is activated for automatic registration.
           
           New in version 1.9.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("registrationResolveMta_ed9c3399", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setRegistrationResolveMta(self, enabled):
        """Set MTA resolution for automatic registration.
           
           If automatic registration is enabled then the point cloud will be created
           with or without MTA resolution depending on this setting.
           
           New in version 1.9.
           
           Arguments:
             enabled (bool): true to enable MTA resolution"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enabled else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setRegistrationResolveMta_dd1e43f5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def unregisterProject(self, storageMedia, project):
        """Unregister project.
           
           Cancel all pending registration tasks for the specified project and
           reset the project to an unregistered state.
           
           New in version 1.1.
           
           Arguments:
             storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
             project (str): project name"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = storageMedia
        inputs[1].s = project
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("unregisterProject_a8a1a86a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def registerProject(self, *args):
        """
           registerProject(storageMedia, project, mode)
               Register all scan positions in a project.
               
               If the project does already have registered scan positions then
               the registration is continued from the last registered scan position.
               
               New in version 1.1.
               
               Arguments:
                 storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
                 project (str): project name
                 mode (RegistrationMode): registration mode
               
               Returns: int
                 ID of background task.

           registerProject(storageMedia, project, mode, resolveMta)
               Register all scan positions in a project.
               
               Overloaded method that allows to disable MTA resolution during project
               registration.
               
               If the project does already have registered scan positions then
               the registration is continued from the last registered scan position.
               
               New in version 1.9.
               
               Arguments:
                 storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
                 project (str): project name
                 mode (RegistrationMode): registration mode
                 resolveMta (bool): false to disable MTA resolution of scans
               
               Returns: int
                 ID of background task.

        """
        if len(args) == 3:
            return self._registerProject_a83df21e(*args)
        elif len(args) == 4:
            return self._registerProject_c3b307a3(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _registerProject_a83df21e(self, storageMedia, project, mode):
        """Register all scan positions in a project.
           
           If the project does already have registered scan positions then
           the registration is continued from the last registered scan position.
           
           New in version 1.1.
           
           Arguments:
             storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
             project (str): project name
             mode (RegistrationMode): registration mode
           
           Returns: int
             ID of background task."""
        inputs = [riconnect.Value() for i in range(0, 3)]
        inputs[0].s = storageMedia
        inputs[1].s = project
        inputs[2].i32 = int(mode)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("registerProject_a83df21e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _registerProject_c3b307a3(self, storageMedia, project, mode, resolveMta):
        """Register all scan positions in a project.
           
           Overloaded method that allows to disable MTA resolution during project
           registration.
           
           If the project does already have registered scan positions then
           the registration is continued from the last registered scan position.
           
           New in version 1.9.
           
           Arguments:
             storageMedia (str): storage media (one of SSD, USB, SDCARD, NAS)
             project (str): project name
             mode (RegistrationMode): registration mode
             resolveMta (bool): false to disable MTA resolution of scans
           
           Returns: int
             ID of background task."""
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].s = storageMedia
        inputs[1].s = project
        inputs[2].i32 = int(mode)
        inputs[3].i32 = 1 if resolveMta else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("registerProject_c3b307a3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def environmentSensorsEnabled(self):
        """Returns true if the information of the environment sensors (temperature, humidity, pressure) are used during data acquisition.
           
           New in version 1.1.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("environmentSensorsEnabled_30ad7a6c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setEnvironmentSensorsEnabled(self, enabled):
        """Enable/Disable the usage of the environment sensor data during data acquisition.
           
           New in version 1.1.
           
           Arguments:
             enabled (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enabled else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setEnvironmentSensorsEnabled_fc5ecab7", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def supportedReflectorSearchModels(self):
        """Return list of supported reflector search models.
           
           The result is returned as JSON object in the following form:
           
           [
             {
               "name": "RIEGL flat reflector 50 mm",
               "shape": "circular disk"
             }
           ]
           
           New in version 1.8.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("supportedReflectorSearchModels_6224229f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setRegistrationBranchPoint(self, branchPoint, anchorPoint):
        """Set a registration branch point for the active project.
           
           When working with scan positions without GNSS information it is often not possible to register
           a scan position if the distance to the previous scan position is too large. In order to solve
           such issues, an anchor point can be specified for a scan position (branch-point). An anchor
           point in this context is a previously registered scan position that is closer to the branch
           point, and therefore more likely to result in a successful registration.
           
           New in version 1.11
           
           Arguments:
             branchPoint (str): name of scan position that should be marked as branch-point
             anchorPoint (str): name of scan position that should be used as the corresponding anchor-point"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = branchPoint
        inputs[1].s = anchorPoint
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setRegistrationBranchPoint_4a6472d5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def registrationBranchPoints(self):
        """Return a JSON object with the specified branch points of the active project.
           
           The result is returned in the following form:
           
           {
             "branchPoint1": {
               "anchor": "anchorPoint1"
            },
             "branchPoint2": {
               "anchor": "anchorPoint2"
             }
           }
           
           New in version 1.11
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("registrationBranchPoints_25199938", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def save(self):
        """Save service properties."""
        with self._lock:
            self._svc.callFunction("__risc__service_properties__save_fn")

    def restore(self):
        """Restore service properties."""
        with self._lock:
            self._svc.callFunction("__risc__service_properties__restore_fn")

    def error(self):
        """The error signal allows services to report problems that are
           unrelated to service calls. It was intended to allow services
           to report hardware related problems. But since most services
           provide some kind of business logic, this signal is rarely used.
           
           Returns: ServiceSignal
           Payload: dict"""
        return self._error

    def horizontalImageOverlapChanged(self):
        """This signal is emitted when the horizontal image overlap changed.
           
           The signal payload is the new image overlap.
           
           Returns: ServiceSignal
           Payload: int"""
        return self._horizontalImageOverlapChanged

    def imageCaptureModeChanged(self):
        """This signal is emitted when the image capture mode changed.
           
           The signal payload is the new capture mode.
           
           Returns: ServiceSignal
           Payload: ImageAcquisitionCaptureMode"""
        return self._imageCaptureModeChanged

    def taskModeChanged(self):
        """This signal is emitted when a task's execution mode changed.
           
           The signal payload contains the task and new task mode.
           
           Returns: ServiceSignal
           Payload: TaskModeChangedPayload"""
        return self._taskModeChanged

    def storeMeasurementStreamChanged(self):
        """This signal is emitted when the measurement stream storage flag changed.
           
           Returns: ServiceSignal
           Payload: bool"""
        return self._storeMeasurementStreamChanged

    def storeMonitorStreamChanged(self):
        """This signal is emitted when the monitor stream storage flag changed.
           
           Returns: ServiceSignal
           Payload: bool"""
        return self._storeMonitorStreamChanged

    def activeUsbStorageDeviceChanged(self):
        """This signal is emitted when the active USB storage device changed.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._activeUsbStorageDeviceChanged

    def activeSdcStorageDeviceChanged(self):
        """This signal is emitted when the active SD-Card storage device changed.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._activeSdcStorageDeviceChanged

    def reflectorSearchSettingsChanged(self):
        """This signal is emitted when the reflector search parameters changed.
           
           Returns: ServiceSignal
           Payload: ReflectorSearchSettings"""
        return self._reflectorSearchSettingsChanged

    def reflectorScanSettingsChanged(self):
        """This signal is emitted when the reflector scan parameters changed.
           
           Returns: ServiceSignal
           Payload: ReflectorScanSettings"""
        return self._reflectorScanSettingsChanged

    def poseEstimationSettingsChanged(self):
        """This signal is emitted when the pose estimation parameters changed.
           
           Returns: ServiceSignal
           Payload: PoseEstimationSettings"""
        return self._poseEstimationSettingsChanged

    def registrationModeChanged(self):
        """This signal is emitted when the point cloud registration mode changed.
           
           New in version 1.1.
           
           Returns: ServiceSignal
           Payload: RegistrationMode"""
        return self._registrationModeChanged

    def registrationResolveMtaChanged(self):
        """This signal is emitted when the MTA resolution of the automatic registration changed.
           
           New in version 1.9.
           
           Returns: ServiceSignal
           Payload: bool"""
        return self._registrationResolveMtaChanged

    def environmentSensorsEnabledChanged(self):
        """This signal is emitted when the environment sensor usage changed.
           
           New in version 1.1.
           
           Returns: ServiceSignal
           Payload: bool"""
        return self._environmentSensorsEnabledChanged

    def registrationBranchPointChanged(self):
        """This signal is emitted when a registration branch point changed.
           
           The signal payload is a JSON object in the following form:
           
           {
              "branchPoint": "ScanPos050",
              "anchorPoint": "ScanPos012"
           }
           
           New in version 1.11
           
           Returns: ServiceSignal
           Payload: str"""
        return self._registrationBranchPointChanged

    def acquisitionStarted(self):
        """This signal is emitted when the data acquisition started.
           
           Returns: ServiceSignal
           Payload: AcquisitionMode"""
        return self._acquisitionStarted

    def acquisitionFinished(self):
        """This signal is emitted when data acquisition finished.
           
           Returns: ServiceSignal
           Payload: ResultCode"""
        return self._acquisitionFinished

    def taskStarted(self):
        """This signal is emitted when an acquisition task started.
           
           The signal payload is a JSON object and contains (at least) the following
           attributes. Additional information contained in the payload is task
           dependent.
           {
              "id": 1,
              "canPause": false,
              "canRestart": false
           }
           
           Attribute description:
               id           the task identifier (see AcquisitionTask)
               canPause     true if the task can be paused; otherwise false
               canRestart   true if the task can be restarted; otherwise false
           
           Returns: ServiceSignal
           Payload: str"""
        return self._taskStarted

    def taskProgress(self):
        """This signal notifies about acquisition task progress changes.
           
           The signal payload is a JSON object and contains (at least) the following
           attributes. Additional information contained in the payload is task
           dependent.
           {
              "id": 1,
              "progress": 90,
              "progresstext": "Image 9 of 10"
           }
           
           Attribute description:
               id           the task identifier (see AcquisitionTask)
               progress     the progress in percent
               progresstext textual progress information (optional)
           
           Returns: ServiceSignal
           Payload: str"""
        return self._taskProgress

    def taskFinished(self):
        """This signal is emitted when an acquisition task finished.
           
           The signal payload is a JSON object and contains (at least) the following
           attributes. Additional information contained in the payload is task
           dependent.
           {
              "id": 1,
              "resultcode": 0
           }
           
           Attribute description:
               id           the task identifier (see AcquisitionTask)
               resultcode   the task's result code (see ResultCode)
           
           Returns: ServiceSignal
           Payload: str"""
        return self._taskFinished

    def taskStateChanged(self):
        """This signal is emitted when an acquisition task changes its state from running to paused and vice versa.
           
           The signal payload is a JSON object and contains the following attributes.
           {
              "id": 1,
              "state": 1
           }
           
           Attribute description:
               id           the task identifier (see AcquisitionTask)
               state        the new task state (see AcquisitionTaskState)
           
           Returns: ServiceSignal
           Payload: str"""
        return self._taskStateChanged

    def backgroundTaskAdded(self):
        """This signal is emitted when a new background task has been added.
           
           The signal payload is a JSON object and contains at least the following attributes.
           {
               "id": 1,
               "type": "RdbCreation",
               "state": "queued",
               "createdAt": "2018-04-04 08:15:03"
           }
           
           Attribute description (backgroundTaskAdded, backgroundTaskChanged, backgroundTaskRemoved):
               id           ID of the background task. Each background task has a unique
                            task identifier that gets assigned when the task is created.
               type         the type of the background task
                            Supported types:
                                RdbCreation
                                Registration
                                ProjectRegistration
               state        the task state.
                            Available states:
                                queued
                                running  (used in backgroundTaskChanged)
                                finished (used in backgroundTaskRemoved)
                                failed   (used in backgroundTaskRemoved)
           
           It is guaranteed that for each backgroundTaskAdded signal there will be a
           backgroundTaskRemoved signal. A task can emit multiple backgroundTaskChanged
           signals. The backgroundTaskChanged signals will be emitted after the
           backgroundTaskAdded signal and before the backgroundTaskRemoved signal.
           
           All signals belonging to the same background task can be identified by
           the task id.
           
           Example:
           {
               "id": 3,
               "type": "ProjectRegistration",
               "state": "queued",
               "createdAt": "2018-04-04 08:00:03",
               "taskInfo": {
                   "project": "Proj01",
                   "storagemedia": "SSD"
               }
           }
           
           New in version 1.1.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._backgroundTaskAdded

    def backgroundTaskChanged(self):
        """This signal is emitted when a background task changed.
           
           The signal payload is a JSON object containing the task's information.
           
           Example:
           {
               "id": 4,
               "type": "RdbCreation",
               "state": "running",
               "createdAt": "2018-04-04 08:00:05",
               "startedAt": "2018-04-04 08:02:03",
               "progress": 3,
               "taskInfo": {
                   "project": "Proj01",
                   "rxpFilePath": "/media/intern/projects/Proj01.PROJ/ScanPos001.SCNPOS/scans/170306_160828.rxp",
                   "scan": "170306_160828",
                   "scanposition": "ScanPos001",
                   "storagemedia": "SSD"
               }
           }
           
           New in version 1.1.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._backgroundTaskChanged

    def backgroundTaskRemoved(self):
        """This signal is emitted when a background task has been removed.
           
           The signal payload is a JSON object containing the task's information.
           
           Example:
           {
               "id": 5,
               "type": "Registration",
               "state": "finished",
               "createdAt": "2018-04-04 08:06:10",
               "startedAt": "2018-04-04 08:06:10",
               "finishedAt": "2018-04-04 08:06:42",
               "taskInfo": {
                   "project": "Proj01",
                   "rdbFilePath": "/media/intern/projects/Proj01.PROJ/ScanPos001.SCNPOS/scans/170414_133333.rdbx",
                   "scan": "170414_133333",
                   "scanposition": "ScanPos001",
                   "storagemedia": "SSD"
               }
           }
           
           New in version 1.1.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._backgroundTaskRemoved
