# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.5"

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


class CaptureInfo(dict):
    def __init__(self, *args, **kwargs):
        self['status'] = None
        self['message'] = None
        self['filename'] = None
        self['filenameThumbnail'] = None
        self['filenameCalibration'] = None
        super().__init__(*args, **kwargs)
    @property
    def status(self):
        return self['status']
    @status.setter
    def status(self, value):
        self['status'] = value
    @property
    def message(self):
        return self['message']
    @message.setter
    def message(self, value):
        self['message'] = value
    @property
    def filename(self):
        return self['filename']
    @filename.setter
    def filename(self, value):
        self['filename'] = value
    @property
    def filenameThumbnail(self):
        return self['filenameThumbnail']
    @filenameThumbnail.setter
    def filenameThumbnail(self, value):
        self['filenameThumbnail'] = value
    @property
    def filenameCalibration(self):
        return self['filenameCalibration']
    @filenameCalibration.setter
    def filenameCalibration(self, value):
        self['filenameCalibration'] = value

class MultiCaptureInfo(dict):
    def __init__(self, *args, **kwargs):
        self['status'] = None
        self['message'] = None
        self['flags'] = None
        self['filename'] = []
        self['filenameThumbnail'] = []
        self['frameAngle'] = []
        self['filenameCalibration'] = None
        super().__init__(*args, **kwargs)
    @property
    def status(self):
        return self['status']
    @status.setter
    def status(self, value):
        self['status'] = value
    @property
    def message(self):
        return self['message']
    @message.setter
    def message(self, value):
        self['message'] = value
    @property
    def flags(self):
        return self['flags']
    @flags.setter
    def flags(self, value):
        self['flags'] = value
    @property
    def filename(self):
        return self['filename']
    @filename.setter
    def filename(self, value):
        self['filename'] = value
    @property
    def filenameThumbnail(self):
        return self['filenameThumbnail']
    @filenameThumbnail.setter
    def filenameThumbnail(self, value):
        self['filenameThumbnail'] = value
    @property
    def frameAngle(self):
        return self['frameAngle']
    @frameAngle.setter
    def frameAngle(self, value):
        self['frameAngle'] = value
    @property
    def filenameCalibration(self):
        return self['filenameCalibration']
    @filenameCalibration.setter
    def filenameCalibration(self, value):
        self['filenameCalibration'] = value

class CalibData(dict):
    def __init__(self, *args, **kwargs):
        self['fileName'] = None
        self['name'] = None
        self['cameraModel'] = None
        self['cameraSerial'] = None
        self['lensModel'] = None
        self['lensSerial'] = None
        self['mountModel'] = None
        self['mountSerial'] = None
        self['fovHorizontal'] = None
        self['fovVertical'] = None
        self['focalLength'] = None
        self['rotation'] = None
        super().__init__(*args, **kwargs)
    @property
    def fileName(self):
        return self['fileName']
    @fileName.setter
    def fileName(self, value):
        self['fileName'] = value
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def cameraModel(self):
        return self['cameraModel']
    @cameraModel.setter
    def cameraModel(self, value):
        self['cameraModel'] = value
    @property
    def cameraSerial(self):
        return self['cameraSerial']
    @cameraSerial.setter
    def cameraSerial(self, value):
        self['cameraSerial'] = value
    @property
    def lensModel(self):
        return self['lensModel']
    @lensModel.setter
    def lensModel(self, value):
        self['lensModel'] = value
    @property
    def lensSerial(self):
        return self['lensSerial']
    @lensSerial.setter
    def lensSerial(self, value):
        self['lensSerial'] = value
    @property
    def mountModel(self):
        return self['mountModel']
    @mountModel.setter
    def mountModel(self, value):
        self['mountModel'] = value
    @property
    def mountSerial(self):
        return self['mountSerial']
    @mountSerial.setter
    def mountSerial(self, value):
        self['mountSerial'] = value
    @property
    def fovHorizontal(self):
        return self['fovHorizontal']
    @fovHorizontal.setter
    def fovHorizontal(self, value):
        self['fovHorizontal'] = value
    @property
    def fovVertical(self):
        return self['fovVertical']
    @fovVertical.setter
    def fovVertical(self, value):
        self['fovVertical'] = value
    @property
    def focalLength(self):
        return self['focalLength']
    @focalLength.setter
    def focalLength(self, value):
        self['focalLength'] = value
    @property
    def rotation(self):
        return self['rotation']
    @rotation.setter
    def rotation(self, value):
        self['rotation'] = value

def _cameraservice_error_decoder(payload):
    return json.loads(payload.decode())
def _cameraservice_finished_decoder(payload):
    return CaptureInfo(json.loads(payload.decode()))
def _cameraservice_multifinished_decoder(payload):
    return MultiCaptureInfo(json.loads(payload.decode()))
def _cameraservice_discovered_decoder(payload):
    return struct.unpack("!I", payload)[0]
def _cameraservice_lost_decoder(payload):
    return struct.unpack("!I", payload)[0]
def _cameraservice_opened_decoder(payload):
    return struct.unpack("!I", payload)[0]
def _cameraservice_closed_decoder(payload):
    return struct.unpack("!I", payload)[0]
def _cameraservice_calibrationchanged_decoder(payload):
    return CalibData(json.loads(payload.decode()))
def _cameraservice_exposuretimelimited_decoder(payload):
    return struct.unpack("!I", payload)[0]

class CameraService(object):
    """"""

    class Error(enum.IntEnum):
        SUCCESS = 0
        UNIMPLEMENTED = 1
        GENERIC_ERROR = 2
        PIMAC_ERROR = 3
        POST_PROCESSING_FAILED = 4
        EXIF_FAILED = 5
        JPEG_FAILED = 6
        PPM_FAILED = 7
        JSON_ERROR = 8
        STORAGE_FAILED = 9
        CONTINUOUS_TRIGGER_FAILED = 10
        RECEIVE_TIMEOUT = 11

    class Flags(enum.IntEnum):
        EXPOSURE_TIME_LIMITED = 1

    class Color(enum.IntEnum):
        COLOR_ORIGINAL = 0
        COLOR_RAINBOW = 1
        COLOR_ELECTRIC = 2

    class AccessLevel(enum.IntEnum):
        ACCESS_BASIC = 0
        ACCESS_ADVANCED = 1
        ACCESS_EXPERT = 2

    class AccessLevelAvailable(enum.IntEnum):
        AVAILABLE_BASIC = 1
        AVAILABLE_ADVANCED = 2
        AVAILABLE_EXPERT = 4

    SUCCESS = Error.SUCCESS
    UNIMPLEMENTED = Error.UNIMPLEMENTED
    GENERIC_ERROR = Error.GENERIC_ERROR
    PIMAC_ERROR = Error.PIMAC_ERROR
    POST_PROCESSING_FAILED = Error.POST_PROCESSING_FAILED
    EXIF_FAILED = Error.EXIF_FAILED
    JPEG_FAILED = Error.JPEG_FAILED
    PPM_FAILED = Error.PPM_FAILED
    JSON_ERROR = Error.JSON_ERROR
    STORAGE_FAILED = Error.STORAGE_FAILED
    CONTINUOUS_TRIGGER_FAILED = Error.CONTINUOUS_TRIGGER_FAILED
    RECEIVE_TIMEOUT = Error.RECEIVE_TIMEOUT
    EXPOSURE_TIME_LIMITED = Flags.EXPOSURE_TIME_LIMITED
    COLOR_ORIGINAL = Color.COLOR_ORIGINAL
    COLOR_RAINBOW = Color.COLOR_RAINBOW
    COLOR_ELECTRIC = Color.COLOR_ELECTRIC
    ACCESS_BASIC = AccessLevel.ACCESS_BASIC
    ACCESS_ADVANCED = AccessLevel.ACCESS_ADVANCED
    ACCESS_EXPERT = AccessLevel.ACCESS_EXPERT
    AVAILABLE_BASIC = AccessLevelAvailable.AVAILABLE_BASIC
    AVAILABLE_ADVANCED = AccessLevelAvailable.AVAILABLE_ADVANCED
    AVAILABLE_EXPERT = AccessLevelAvailable.AVAILABLE_EXPERT

    def __init__(self, address):
        self._svc = riconnect.Service("CameraService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_cameraservice_error_decoder)
        self._started = ServiceSignal(self._svc, "started")
        self._ready = ServiceSignal(self._svc, "ready")
        self._finished = ServiceSignal(self._svc, "finished", decoderFn=_cameraservice_finished_decoder)
        self._multiFinished = ServiceSignal(self._svc, "multiFinished", decoderFn=_cameraservice_multifinished_decoder)
        self._discovered = ServiceSignal(self._svc, "discovered", decoderFn=_cameraservice_discovered_decoder)
        self._lost = ServiceSignal(self._svc, "lost", decoderFn=_cameraservice_lost_decoder)
        self._opened = ServiceSignal(self._svc, "opened", decoderFn=_cameraservice_opened_decoder)
        self._closed = ServiceSignal(self._svc, "closed", decoderFn=_cameraservice_closed_decoder)
        self._calibrationChanged = ServiceSignal(self._svc, "calibrationChanged", decoderFn=_cameraservice_calibrationchanged_decoder)
        self._exposureTimeLimited = ServiceSignal(self._svc, "exposureTimeLimited", decoderFn=_cameraservice_exposuretimelimited_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def rotation(self):
        """Get image rotation in degrees
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("rotation_a5c02a3b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def color(self):
        """Get color mode for FLIR cameras (default is COLOR_ORIGINAL)
           
           Returns: Color"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("color_70dda5df", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(CameraService.Color(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setColor(self, col):
        """Set color mode for FLIR cameras
           
           Arguments:
             col (Color): Color definition of enum Color"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(col)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setColor_0a85522c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def filename(self):
        """Get actual filename (default is "")
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("filename_435ed7e9", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setFilename(self, filename):
        """Set actual filename
           
           Arguments:
             filename (str): Filename for image acquisition"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = filename
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setFilename_2613725a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def filenameThumbnail(self):
        """Get actual filename of thumbnail (default is "")
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("filenameThumbnail_5b1e15d4", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setFilenameThumbnail(self, filename):
        """Set actual filename of thumbnail
           
           Arguments:
             filename (str): Filename of thumbnail for image acquisition"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = filename
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setFilenameThumbnail_d1550a49", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def widthThumbnail(self):
        """Get actual thumbnail width (default = 320)
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("widthThumbnail_1677e7d2", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setWidthThumbnail(self, width):
        """Set actual thumbnail width
           
           Arguments:
             width (int): Width in pixel"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = width
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setWidthThumbnail_72ff7e63", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def heightThumbnail(self):
        """Get actual thumbnail height (default = 200)
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("heightThumbnail_ac1b6667", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setHeightThumbnail(self, height):
        """Set actual thumbnail height
           
           Arguments:
             height (int): Height in pixel"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = height
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setHeightThumbnail_d1fe6e64", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def list(self):
        """List all discovered cameras
           
           Returns: list(int)
             List of unique IDs for each camera"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("list_8b154ab8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        if len(rtransfers[0]) > 0:
            outputs.append(struct.unpack("!{0}I".format(int(len(rtransfers[0])/4)), rtransfers[0]))
        else:
            outputs.append(())
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def model(self, cam_id):
        """Get model name of camera
           
           Arguments:
             cam_id (int): The unique ID of the camera
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = cam_id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("model_a9836cf1", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def manufacturer(self, cam_id):
        """Get manufacturer of camera
           
           Arguments:
             cam_id (int): The unique ID of the camera
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = cam_id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("manufacturer_c8a5241d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def serialNumber(self, cam_id):
        """Get serial number of camera
           
           Arguments:
             cam_id (int): The unique ID of the camera
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = cam_id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("serialNumber_8f7649f2", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def deviceVersion(self, cam_id):
        """Get device version of camera
           
           Arguments:
             cam_id (int): The unique ID of the camera
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = cam_id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("deviceVersion_0862e414", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def cameraInfo(self, cam_id):
        """Get camera information
           
           Arguments:
             cam_id (int): The unique ID of the camera
           
           Returns: tuple(str, str, str, str)
             1: Manufacturer of camera
             2: Model name of camera
             3: Serial number of camera
             4: Device version of camera"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = cam_id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("cameraInfo_e88f08d6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(rvalues[0].s)
        outputs.append(rvalues[1].s)
        outputs.append(rvalues[2].s)
        outputs.append(rvalues[3].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def isThermal(self):
        """Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isThermal_2f7c04b0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def isHyperspectral(self):
        """Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isHyperspectral_e822a6f5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def isCaptureMultiSupported(self):
        """Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isCaptureMultiSupported_103d307f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def fieldOfView(self):
        """Get cameras FOV FOV will already be rotated correctly according to mounting
           
           Returns: tuple(float, float)
             1: Horizontal field of view in degrees
             2: Vertical field of view in degrees"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("fieldOfView_64d2b75f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(rvalues[0].f)
        outputs.append(rvalues[1].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def focalLength(self):
        """Get focal length in mm
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("focalLength_8b76e0aa", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def sensorWidth(self):
        """Get cameras sensor width in pixel
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("sensorWidth_c7f70d6b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def sensorHeight(self):
        """Get cameras sensor height in pixel
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("sensorHeight_e7c2d910", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def open(self, cam_id):
        """Open and initialize camera
           
           Will close any still open camera.
           This method will raise an exception if capture or processing are still in progress for camera to be closed!
           
           Arguments:
             cam_id (int): The unique ID of the camera"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = cam_id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("open_9460d43d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def close(self):
        """Close camera
           
           This method will raise an exception if capture or processing are still in progress!"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("close_716f6b30", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def activeCamera(self):
        """Get active camera
           
           Resturns 0 if no active camera is set
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("activeCamera_0c9ecc6f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def busy(self):
        """Check if camera is busy, returns false if not
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("busy_8bc1b2f8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def pendingImages(self):
        """Get pending images count
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("pendingImages_42741204", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def releasedImages(self):
        """Get released images count
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("releasedImages_55a8fb98", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def capture(self):
        """Capture a single image (triggered by software)"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("capture_d7ba9bbf", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def captureMulti(self, maxExposureTime):
        """Capture multiple images (triggered by scanner)
           
           Arguments:
             maxExposureTime (int): Exposure time in microseconds"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = maxExposureTime
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("captureMulti_a03e4939", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=30.0)
        
    def abort(self):
        """Abort image capture"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("abort_5bb94a1c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def bufferInfo(self, thumbnail):
        """Get buffer information
           
           Arguments:
             thumbnail (bool): True if thumbnail should be used
           
           Returns: tuple(int, int, int, int, str)
             1: Pixel format as defined in PIMAC
             2: Width of the image
             3: Height of the image
             4: Length of the buffer
             5: A description string"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if thumbnail else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("bufferInfo_7145495b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(rvalues[0].u32)
        outputs.append(rvalues[1].u32)
        outputs.append(rvalues[2].u32)
        outputs.append(rvalues[3].u32)
        outputs.append(rvalues[4].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def multiBufferInfo(self, thumbnail, index):
        """Get buffer information with index
           
           Arguments:
             thumbnail (bool): True if thumbnail should be used
             index (int): Index of image to get buffer information from
           
           Returns: tuple(int, int, int, int, str)
             1: Pixel format as defined in PIMAC
             2: Width of the image
             3: Height of the image
             4: Length of the buffer
             5: A description string"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = 1 if thumbnail else 0
        inputs[1].u32 = index
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("multiBufferInfo_7bbd7338", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(rvalues[0].u32)
        outputs.append(rvalues[1].u32)
        outputs.append(rvalues[2].u32)
        outputs.append(rvalues[3].u32)
        outputs.append(rvalues[4].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def captureBuffer(self, thumbnail):
        """Get buffer of last capture
           
           Arguments:
             thumbnail (bool): True if thumbnail should be used
           
           Returns: bytes
             Data of image"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if thumbnail else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("captureBuffer_fdd9ea88", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        outputs.append(rtransfers[0])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def multiCaptureBuffer(self, thumbnail, index):
        """Get buffer of last capture with index
           
           Arguments:
             thumbnail (bool): True if thumbnail should be used
             index (int): Index of image to get buffer
           
           Returns: bytes
             Data of image"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = 1 if thumbnail else 0
        inputs[1].u32 = index
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("multiCaptureBuffer_f715c9c1", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        outputs.append(rtransfers[0])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def captureFilename(self, thumbnail):
        """Get filename of last capture
           
           Arguments:
             thumbnail (bool): True if thumbnail should be used
           
           Returns: str
             Filename of image"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if thumbnail else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("captureFilename_03e365eb", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def multiCaptureFilename(self, thumbnail, index):
        """Get filename of last capture with index
           
           Arguments:
             thumbnail (bool): True if thumbnail should be used
             index (int): Index of image to get filename from
           
           Returns: str
             Filename of image"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = 1 if thumbnail else 0
        inputs[1].u32 = index
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("multiCaptureFilename_e4d5fc29", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def xml(self):
        """Get XML description for camera
           
           Returns: bytes
             Content of XML description"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("xml_75e01904", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        outputs.append(rtransfers[0])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def readRegister(self, address):
        """Read value from register
           
           Arguments:
             address (int): Memory address of register
           
           Returns: int"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i64 = address
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("readRegister_caf73ac8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].i64)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def writeRegister(self, address, value):
        """Write value to register
           
           Arguments:
             address (int): Memory address of register
             value (int): Value to set into register"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i64 = address
        inputs[1].i64 = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("writeRegister_dedad9af", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def readMemory(self, address, length):
        """Read value from memory
           
           Arguments:
             address (int): Memory address to read
             length (int): Length to read
           
           Returns: bytes
             Data read"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i64 = address
        inputs[1].i64 = length
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("readMemory_faa7131f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        outputs.append(rtransfers[0])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def writeMemory(self, address, buffer):
        """Write value to memory
           
           Arguments:
             address (int): Memory address of register
             buffer (bytes): Data to write"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i64 = address
        inputTransfers = []
        if isinstance(buffer, bytes) or isinstance(buffer, bytearray):
            tb_buffer = buffer
        else:
            tb_buffer = struct.pack("!{0}B".format(len(buffer)), *buffer)
        inputTransfers.append(tb_buffer)
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("writeMemory_1e03116d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def availableAccessLevels(self):
        """Get bit mask of available access levels
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("availableAccessLevels_20dc6326", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=30.0)
        outputs = []

        outputs.append(rvalues[0].i32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def propertyDescription(self, level):
        """Get a description for all properties with given access level
           
           Arguments:
             level (AccessLevel): Access level for properties to retrieve
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(level)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("propertyDescription_1c0b8cc5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=30.0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setProperty(self, name, value):
        """Set a property value by name
           
           Arguments:
             name (str): Name of property to set
             value (str): Property to set as string"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = name
        inputs[1].s = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setProperty_6cee414a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def property(self, name):
        """Get a property value by name
           
           Arguments:
             name (str): Name of property to get
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("property_425b08bd", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def listCalibrations(self, cam_id):
        """List all calibrations
           
           Arguments:
             cam_id (int): The unique ID of the camera
           
           Returns: list(str)
             List of calibrations (returns only valid for active camera)"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = cam_id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("listCalibrations_5f5516bb", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        _ot_calibs = []
        _otsplits = rtransfers[0][0:-1].split(b"\0") if rtransfers[0] else []
        for part in _otsplits:
            _ot_calibs.append(part.decode())
        outputs.append(_ot_calibs)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setCalibration(self, calib):
        """Set a calibration
           
           Arguments:
             calib (str): Name of a calibration"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = calib
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setCalibration_b088b454", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def calibration(self):
        """Get a calibration
           
           returns empty string if no calibration is set
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("calibration_0bf719df", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def calibrationData(self):
        """Get a calibration
           
           raises error if no calibration is set
           
           Returns: CalibData"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("calibrationData_4c29c8fb", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(CalibData(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def calculateHistogram(self, filename, histogram):
        """Calculate Histogram for given file
           
           Arguments:
             filename (str): Name of image file to calculate histogram for
             histogram (list(int)): Vector of gray intensity values (red * 0.34 + green * 0.5 + blue * 0.15)
           
           Returns: list(int)
             Vector of gray intensity values (red * 0.34 + green * 0.5 + blue * 0.15)"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = filename
        inputTransfers = []
        if isinstance(histogram, bytes) or isinstance(histogram, bytearray):
            tb_histogram = histogram
        else:
            tb_histogram = struct.pack("!{0}I".format(len(histogram)), *histogram)
        inputTransfers.append(tb_histogram)
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("calculateHistogram_b0e8a543", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        if len(rtransfers[0]) > 0:
            outputs.append(struct.unpack("!{0}I".format(int(len(rtransfers[0])/4)), rtransfers[0]))
        else:
            outputs.append(())
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def restoreFactory(self):
        """"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("restoreFactory_4f21f459", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
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

    def started(self):
        """Emitted as soon capture starts
           
           Returns: ServiceSignal"""
        return self._started

    def ready(self):
        """Emitted as soon as capture is ready again (image acquired but maybe not transferred)
           
           Returns: ServiceSignal"""
        return self._ready

    def finished(self):
        """Emitted as soon as capture of single image has finished and image is ready (not busy anymore)
           
           Returns: ServiceSignal
           Payload: CaptureInfo"""
        return self._finished

    def multiFinished(self):
        """Emitted as soon as capture of multiple images has finished and images are ready (not busy anymore)
           
           Returns: ServiceSignal
           Payload: MultiCaptureInfo"""
        return self._multiFinished

    def discovered(self):
        """Emitted as soon as camera has been discovered
           
           Returns: ServiceSignal
           Payload: int"""
        return self._discovered

    def lost(self):
        """Emitted as soon as camera has been lost
           
           Returns: ServiceSignal
           Payload: int"""
        return self._lost

    def opened(self):
        """Emitted as soon as camera was opened
           
           Returns: ServiceSignal
           Payload: int"""
        return self._opened

    def closed(self):
        """Emitted as soon as camera was closed
           
           Returns: ServiceSignal
           Payload: int"""
        return self._closed

    def calibrationChanged(self):
        """Emitted as soon as calibration changes
           
           Returns: ServiceSignal
           Payload: CalibData"""
        return self._calibrationChanged

    def exposureTimeLimited(self):
        """Emitted as soon as exposure time is limited
           
           Returns: ServiceSignal
           Payload: int"""
        return self._exposureTimeLimited
