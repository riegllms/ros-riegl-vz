# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "0.1"

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


def _rtktrajectoryservice_error_decoder(payload):
    return json.loads(payload.decode())
def _rtktrajectoryservice_trajectorydatasignal_decoder(payload):
    return payload.decode()
def _rtktrajectoryservice_imudatasignal_decoder(payload):
    return payload.decode()

class RtkTrajectoryService(object):
    """"""

    def __init__(self, address):
        self._svc = riconnect.Service("RtkTrajectoryService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_rtktrajectoryservice_error_decoder)
        self._trajectoryDataSignal = ServiceSignal(self._svc, "trajectoryDataSignal", decoderFn=_rtktrajectoryservice_trajectorydatasignal_decoder)
        self._imuDataSignal = ServiceSignal(self._svc, "imuDataSignal", decoderFn=_rtktrajectoryservice_imudatasignal_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def setCfg(self, cfg):
        """Set rtk-trajectory config string in json format (see documentation of rtk-trajectory tool).
           
           Arguments:
             cfg (str): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = cfg
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setCfg_95efda07", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def start(self):
        """Start trajectory calc."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("start_ea2b2676", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def restart(self):
        """Restart trajectory calc."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("restart_8a7ef1c3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def stop(self):
        """Stop trajectory calc."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("stop_ef399b2d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def imuStart(self):
        """Start IMU data calc."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imuStart_d27d5184", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def imuRestart(self):
        """Restart IMU data calc."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imuRestart_9c4212de", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def imuStop(self):
        """Stop IMU data calc."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("imuStop_9a92e9bd", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
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
    def trajectoryDataSignal(self):
        """Signals trajectory data in json format.
           
           Example Data:
           {
             "riegl.pof_height": 362.98529557240096,
             "riegl.pof_latitude": 48.65822231913168,
            "riegl.pof_longitude": 15.664063094534322,
            "riegl.pof_pitch": 3.797405958175659,
            "riegl.pof_roll": -0.8947142362594604,
            "riegl.pof_timestamp": 48264.2167147,
            "riegl.pof_xyz": [-0.052748819103708405,-0.006008374390010213,-0.21952207721133926],
            "riegl.pof_yaw": 171.48678588867188
           }
           
           Returns: ServiceSignal
           Payload: str"""
        return self._trajectoryDataSignal
    def imuDataSignal(self):
        """Signals IMU data in json format.
           
           Example Data:
           {
             "imu_index":0,
             "riegl.accelerometer":[-0.031407772105840094,0.033759927625578404,9.821126413004606],
             "riegl.accelerometer_raw":[-16496,-1,355],
             "riegl.barometric_height_amsl":0.0,
             "riegl.data_acquisition_active":0,
             "riegl.frame_angle_coarse":268.88893054780505,
             "riegl.frame_scan_active":0,
             "riegl.gyroscope":[-0.07245045564734892,0.0024131052796165626,0.04303260997408862],
             "riegl.gyroscope_raw":[26,-368,-48],
             "riegl.line_scan_active":0,
             "riegl.magnetic_field_sensor":[-6.14794618055051,7.068926504763107,-24.468684772787213],
             "riegl.magnetic_field_sensor_raw":[-6461,7175,4200],
             "riegl.temperature":42.12143053327288,
             "riegl.timestamp":495.1513484403745
           }
           
           Returns: ServiceSignal
           Payload: str"""
        return self._imuDataSignal
