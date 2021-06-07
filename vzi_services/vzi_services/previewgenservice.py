# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.1"

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


def _previewgenservice_error_decoder(payload):
    return json.loads(payload.decode())
def _previewgenservice_reflectanceminimumchanged_decoder(payload):
    return struct.unpack("!f", payload)[0]
def _previewgenservice_reflectancethresholdchanged_decoder(payload):
    return struct.unpack("!f", payload)[0]
def _previewgenservice_finished_decoder(payload):
    return struct.unpack("!B", payload)[0]

class PreviewgenService(object):
    """The preview generation service.
       
       The preview generation service is used to create preview images of a running
       scan acquisition. When the preview generation is started a connection to
       the monitoring data stream is created (via RDTP) and the incoming
       laser shot and echo packages are analyzed to create a 2D preview of the
       scan data."""

    class ErrorCode(enum.IntEnum):
        RXP_STREAM_ERROR = 20000

    RXP_STREAM_ERROR = ErrorCode.RXP_STREAM_ERROR

    def __init__(self, address):
        self._svc = riconnect.Service("PreviewgenService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_previewgenservice_error_decoder)
        self._reflectanceMinimumChanged = ServiceSignal(self._svc, "reflectanceMinimumChanged", decoderFn=_previewgenservice_reflectanceminimumchanged_decoder)
        self._reflectanceThresholdChanged = ServiceSignal(self._svc, "reflectanceThresholdChanged", decoderFn=_previewgenservice_reflectancethresholdchanged_decoder)
        self._started = ServiceSignal(self._svc, "started")
        self._finished = ServiceSignal(self._svc, "finished", decoderFn=_previewgenservice_finished_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def generatePreview(self):
        """Start preview generation."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("generatePreview_098cffb6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def isRunning(self):
        """Return true if preview generation is running.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isRunning_39044c41", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def cancel(self):
        """Cancel the running preview generation."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("cancel_10aec353", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def abort(self):
        """Forcefully stop the running preview generation.
           
           Calling this function will terminate the internal worker thread responsible
           of generating the preview image. This function should only be called if
           cancel() has no effect."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("abort_5bb94a1c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getPreview(self, mipLevel):
        """Return the current preview image.
           
           If the requested mipmap level does not exist, then the next available level
           is used.
           
           The following Python code snipped shows how to fetch the preview and store it to disk.
           
               from previewgenservice import PreviewgenService
               from PIL import Image
           
               svc = PreviewgenService("localhost:20000")
               w, h, sw, sh, d = svc.getPreview(0)
               img = Image.frombytes("RGB", (w, h), d)
               img = img.resize((round(w*sw), round(h*sh)), Image.BILINEAR)
               img.save("/tmp/preview.png")
           
           Arguments:
             mipLevel (int): the mipmap level of the image (not really used and should always be 0)
           
           Returns: tuple(int, int, float, float, bytes)
             1: The image width in pixels.
             2: The image height in pixels.
             3: Image width scale factor.
             4: Image height scale factor.
             5: The image data (24 bit per pixel, RGB)."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = mipLevel
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getPreview_a9ce686f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        outputs.append(rvalues[0].u32)
        outputs.append(rvalues[1].u32)
        outputs.append(rvalues[2].f)
        outputs.append(rvalues[3].f)
        outputs.append(rtransfers[0])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getReflectance(self, mipLevel):
        """Return the reflectance data of the current preview.
           
           Values:
           0 - 250 ... the reflectance values (scaled from reflectance_min to reflectance_threshold).
                       To calculate the refelctance in dB the following formula can be used:
                       reflDb = reflectanceMinimum + dataValue/250 * (reflectanceThreshold-reflectanceMinimum)
           254     ... a reflector
           255     ... lasershot with no echo signal
           
           Arguments:
             mipLevel (int): the mipmap level of the image (not really used and should always be 0)
           
           Returns: tuple(int, int, float, float, bytes)
             1: The image width in pixels.
             2: The image height in pixels.
             3: Image width scale factor.
             4: Image height scale factor.
             5: The reflectance data (8 bit per pixel)."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = mipLevel
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getReflectance_7de320db", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        outputs.append(rvalues[0].u32)
        outputs.append(rvalues[1].u32)
        outputs.append(rvalues[2].f)
        outputs.append(rvalues[3].f)
        outputs.append(rtransfers[0])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def mipLevelCount(self):
        """Return the number of available preview image mipmap levels.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mipLevelCount_31227146", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def imageWidth(self):
        """Return the used image width.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imageWidth_fd1d69f1", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setImageWidth(self, value):
        """Set the used image width.
           
           Arguments:
             value (int): the new image width"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setImageWidth_987d8050", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def imageHeight(self):
        """Return the used image height.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imageHeight_135e606f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setImageHeight(self, value):
        """Set the used image height.
           
           Arguments:
             value (int): the new image height."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setImageHeight_79d1e90e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def imageWidthScale(self):
        """Return the scale factor for the image width.
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imageWidthScale_673163f0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setImageWidthScale(self, value):
        """Set the scale factor for the image width.
           
           Scales the created preview image before storing it to disk. The used
           image size (width and height) is most often calculated from the line and
           frame angle increments, resulting in an image having a different aspect
           ratio than the field of view of the scan. Setting a width and height
           scale can be used to scale the generated image before writing it to
           disk, resulting in a preview having the same aspect ratio as the scan
           pattern.
           
           Arguments:
             value (float): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].f = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setImageWidthScale_a1c81949", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def imageHeightScale(self):
        """Return the scale factor for the image height.
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imageHeightScale_421c6970", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setImageHeightScale(self, value):
        """Set the scale factor for the image height.
           
           Scales the created preview image before storing it to disk. The used
           image size (width and height) is most often calculated from the line and
           frame angle increments, resulting in an image having a different aspect
           ratio than the field of view of the scan. Setting a width and height
           scale can be used to scale the generated image before writing it to
           disk, resulting in a preview having the same aspect ratio as the scan
           pattern.
           
           Arguments:
             value (float): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].f = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setImageHeightScale_66f98404", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def reflectanceMinimum(self):
        """Return the minimum reflectance in dB.
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("reflectanceMinimum_c87364d0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setReflectanceMinimum(self, value):
        """Set the minimum reflectance in dB.
           
           Echoes with a reflectance smaller or equal than the specified value do get
           the lowest gray scale color value applied.
           
           Arguments:
             value (float): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].f = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setReflectanceMinimum_49a247ea", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def reflectanceThreshold(self):
        """Return the reflectance threshold in dB.
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("reflectanceThreshold_72a8fad8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setReflectanceThreshold(self, value):
        """Set the reflectance threshold in dB.
           
           Echoes with a reflectance higher than the specified value are treated
           as reflectors.
           
           Arguments:
             value (float): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].f = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setReflectanceThreshold_fcc94fa1", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def mirrorX(self):
        """Return true of the image is mirrored along the X (frame anlge) axis.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mirrorX_365909d7", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMirrorX(self, value):
        """If set to true then the generated preview is mirrored along the frame angle axis.
           
           Arguments:
             value (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if value else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMirrorX_6d70ec71", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def mirrorY(self):
        """Return true of the image is mirrored along the Y (line angle) axis.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mirrorY_c5bf6aaf", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMirrorY(self, value):
        """If set to true then the generated preview is mirrored along the line angle axis.
           
           Arguments:
             value (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if value else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMirrorY_61a18298", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def monitoringStreamUri(self):
        """Return the URI of the monitoring data stream.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("monitoringStreamUri_a288d6e3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMonitoringStreamUri(self, value):
        """Set the URI used to connect to the monitoring data stream.
           
           Arguments:
             value (str): the new data stream URI"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMonitoringStreamUri_3b0a401b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def previewFilePath(self):
        """Return the absolute file path of the file the generated preview gets written to.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("previewFilePath_dd11fc6e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setPreviewFilePath(self, value):
        """Set the absolute file path of the file the generated preview gets written to.
           
           If empty then the preview image is not stored to disk.
           
           Arguments:
             value (str): the file path of the result image"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setPreviewFilePath_86f894d7", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
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

    def reflectanceMinimumChanged(self):
        """This signal is emitted when the minimum reflectance changed.
           
           Returns: ServiceSignal
           Payload: float"""
        return self._reflectanceMinimumChanged

    def reflectanceThresholdChanged(self):
        """This signal is emitted when the reflectance threshold changed.
           
           Returns: ServiceSignal
           Payload: float"""
        return self._reflectanceThresholdChanged

    def started(self):
        """This signal is emitted when the preview generation started.
           
           Returns: ServiceSignal"""
        return self._started

    def finished(self):
        """This signal is emitted when the preview generation finished.
           
           The signal payload represents the following states:
               0 = success
               1 = preview generation was canceled
               2 = preview generation was aborted
               3 = preview generation failed due to some error
           
           Returns: ServiceSignal
           Payload: int"""
        return self._finished
