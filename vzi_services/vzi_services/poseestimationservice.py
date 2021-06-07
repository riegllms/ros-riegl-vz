# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.4"

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


class FinishedPayload(dict):
    def __init__(self, *args, **kwargs):
        self['success'] = None
        self['errorMessage'] = None
        self['jsonData'] = None
        super().__init__(*args, **kwargs)
    @property
    def success(self):
        return self['success']
    @success.setter
    def success(self, value):
        self['success'] = value
    @property
    def errorMessage(self):
        return self['errorMessage']
    @errorMessage.setter
    def errorMessage(self, value):
        self['errorMessage'] = value
    @property
    def jsonData(self):
        return self['jsonData']
    @jsonData.setter
    def jsonData(self, value):
        self['jsonData'] = value

class MotionChange(dict):
    def __init__(self, *args, **kwargs):
        self['rotChange'] = None
        self['accChange'] = None
        super().__init__(*args, **kwargs)
    @property
    def rotChange(self):
        return self['rotChange']
    @rotChange.setter
    def rotChange(self, value):
        self['rotChange'] = value
    @property
    def accChange(self):
        return self['accChange']
    @accChange.setter
    def accChange(self, value):
        self['accChange'] = value

def _poseestimationservice_error_decoder(payload):
    return json.loads(payload.decode())
def _poseestimationservice_progress_decoder(payload):
    return struct.unpack("!i", payload)[0]
def _poseestimationservice_finished_decoder(payload):
    return FinishedPayload(json.loads(payload.decode()))
def _poseestimationservice_imupositionschanged_decoder(payload):
    return struct.unpack("!B", payload)[0]
def _poseestimationservice_imumeasurementschanged_decoder(payload):
    return struct.unpack("!B", payload)[0]
def _poseestimationservice_motiondetected_decoder(payload):
    return MotionChange(json.loads(payload.decode()))
def _poseestimationservice_motiondetectorstatechanged_decoder(payload):
    return struct.unpack("!?", payload)[0]
def _poseestimationservice_motiondetectorsensitivitychanged_decoder(payload):
    return struct.unpack("!i", payload)[0]

class PoseEstimationService(object):
    """The pose estimation service.
       
       The pose estimation service is used to estimation the scanner's pose."""

    class MotionDetectorSensitivity(enum.IntEnum):
        SENSITIVITY_LOW = 0
        SENSITIVITY_MEDIUM = 1
        SENSITIVITY_HIGH = 2

    SENSITIVITY_LOW = MotionDetectorSensitivity.SENSITIVITY_LOW
    SENSITIVITY_MEDIUM = MotionDetectorSensitivity.SENSITIVITY_MEDIUM
    SENSITIVITY_HIGH = MotionDetectorSensitivity.SENSITIVITY_HIGH

    def __init__(self, address):
        self._svc = riconnect.Service("PoseEstimationService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_poseestimationservice_error_decoder)
        self._started = ServiceSignal(self._svc, "started")
        self._progress = ServiceSignal(self._svc, "progress", decoderFn=_poseestimationservice_progress_decoder)
        self._finished = ServiceSignal(self._svc, "finished", decoderFn=_poseestimationservice_finished_decoder)
        self._imuPositionsChanged = ServiceSignal(self._svc, "imuPositionsChanged", decoderFn=_poseestimationservice_imupositionschanged_decoder)
        self._imuMeasurementsChanged = ServiceSignal(self._svc, "imuMeasurementsChanged", decoderFn=_poseestimationservice_imumeasurementschanged_decoder)
        self._motionDetected = ServiceSignal(self._svc, "motionDetected", decoderFn=_poseestimationservice_motiondetected_decoder)
        self._motionDetectorStateChanged = ServiceSignal(self._svc, "motionDetectorStateChanged", decoderFn=_poseestimationservice_motiondetectorstatechanged_decoder)
        self._motionDetectorSensitivityChanged = ServiceSignal(self._svc, "motionDetectorSensitivityChanged", decoderFn=_poseestimationservice_motiondetectorsensitivitychanged_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def setProjectPosition(self, latitude, longitude, altitude):
        """Set the project location used during pose estimation.
           
           Arguments:
             latitude (float): the latitude coordinate
             longitude (float): the longitude coordinate
             altitude (float): the altitude coordinate"""
        inputs = [riconnect.Value() for i in range(0, 3)]
        inputs[0].d = latitude
        inputs[1].d = longitude
        inputs[2].d = altitude
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setProjectPosition_03de9582", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def setImuPositions(self, pos):
        """Set the number of scan position where measurements should be taken.
           
           Arguments:
             pos (int): the number of position (range 1 to 64)"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = pos
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setImuPositions_db0bd3bc", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def setImuMeasurements(self, meas):
        """Set the number of measurements that should be taken at each position.
           
           Arguments:
             meas (int): the number of measurements (range 1 to 128)"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = meas
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setImuMeasurements_2484cc1c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def projectPosition(self):
        """Return the current position.
           
           The data is returned as JSON string in the following format:
           {
             "latitude": 0,
             "longitude": 0,
             "height": 0
           }
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("projectPosition_043afea8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def imuPositions(self):
        """Return the number of positions where measurements should be taken.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imuPositions_b8898f96", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def imuMeasurements(self):
        """Return the number of measurements that should be taken at each position.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imuMeasurements_56cc2748", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def startPoseEstimation(self):
        """Start the pose estimation."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startPoseEstimation_a5ae43d6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def startPoseEstimationFromScan(self):
        """Start the pose estimation that estimates the pose from the data acquired during a running scan acquisition."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startPoseEstimationFromScan_bb69bbed", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def stop(self):
        """Stop any running pose estimation."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("stop_ef399b2d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def isBusy(self):
        """Return true if a pose estimation is running.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isBusy_3fa264f2", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def lastPoseEstimate(self):
        """Return the results of the last pose estimation.
           
           The data is returned as JSON string in the following format:
           
           {
             "mrot": [-0.008, -0.008, -0.234],
             "mrot_conf": [27.295, 27.295, 27.295],
             "pitch": -0.77599123360519817,
             "pitch_conf": 0.01958979896512429,
             "roll": -0.29453755237846263,
             "roll_conf": 0.01970221922262999,
             "sscale": 0.99596467966642011,
             "sscale_conf": 0.027227021762934699,
             "uoffset": [-679.753, 28.250, 1173.9407],
             "uoffset_conf": [38.231, 227.133, 106.638],
             "uscale": 1.002232035223716,
             "uscale_conf": 0.015286503296711948,
             "yaw": 106.51162781493518,
             "yaw_conf": 1.279995362575793
           }
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("lastPoseEstimate_6c40e6ed", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def lastPoseImuMeasurements(self):
        """Return the IMU measurements of the last pose estimation.
           
           The measurements are returned as CSV file with one measurement per line.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("lastPoseImuMeasurements_63272a3d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def estimateTime(self, meas, pos):
        """Return the estimated time (in seconds) for a pose estimation with the specified number of measurements and positions.
           
           Arguments:
             meas (int): number of measurements (per position)
             pos (int): number of positions
           
           Returns: float"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].u32 = meas
        inputs[1].u32 = pos
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("estimateTime_ce961959", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def startMotionDetector(self):
        """Start the motion detector.
           
           The motion detector monitors the IMU data and tries to detect a movement
           of the scanner. If a motion is detected then the corresponding signal is
           emitted and the motion detector is stopped."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("startMotionDetector_c687503e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def stopMotionDetector(self):
        """Stop the motion detector."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("stopMotionDetector_8b47f715", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def isMotionDetectorRunning(self):
        """Return true if the motion detector is running.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isMotionDetectorRunning_9e7ecfc1", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMotionDetectorSensitivity(self, value):
        """Set the sensitivity of the motion detector.
           
           Arguments:
             value (MotionDetectorSensitivity): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(value)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMotionDetectorSensitivity_64dc02c8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def motionDetectorSensitivity(self):
        """Return the sensitivity of the motion detector.
           
           Returns: MotionDetectorSensitivity"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("motionDetectorSensitivity_e8a6f238", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(PoseEstimationService.MotionDetectorSensitivity(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setShockDetectorThresholds(self, factor_threshold, peak2peak_threshold):
        """Set the sensitivity of the shock detector.
           
           Arguments:
             factor_threshold (int): 
             peak2peak_threshold (int): """
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = factor_threshold
        inputs[1].i32 = peak2peak_threshold
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setShockDetectorThresholds_2628d169", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
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
        """This signal is emitted when the pose estimation started.
           
           Returns: ServiceSignal"""
        return self._started

    def progress(self):
        """This signal notifies about the progress of the running pose estimation.
           
           The signal payload contains the progress in percent.
           
           Returns: ServiceSignal
           Payload: int"""
        return self._progress

    def finished(self):
        """This signal is emitted when the pose estimation finished.
           
           The signal payload contains information about the succes or failure.
           
           Returns: ServiceSignal
           Payload: FinishedPayload"""
        return self._finished

    def imuPositionsChanged(self):
        """This signal is emitted when the number of positions changed.
           
           Returns: ServiceSignal
           Payload: int"""
        return self._imuPositionsChanged

    def imuMeasurementsChanged(self):
        """This signal is emitted when the number of measurements changed.
           
           Returns: ServiceSignal
           Payload: int"""
        return self._imuMeasurementsChanged

    def motionDetected(self):
        """This signal is emitted when a motion was detected by the motion detector.
           
           Returns: ServiceSignal
           Payload: MotionChange"""
        return self._motionDetected

    def motionDetectorStateChanged(self):
        """This signal is emitted when the motion detector changed its state.
           
           The signal payload is true if the motion detector is running, and false
           otherwise.
           
           Returns: ServiceSignal
           Payload: bool"""
        return self._motionDetectorStateChanged

    def motionDetectorSensitivityChanged(self):
        """This signal is emitted when the motion detector sensitivity changed.
           
           Returns: ServiceSignal
           Payload: MotionDetectorSensitivity"""
        return self._motionDetectorSensitivityChanged
