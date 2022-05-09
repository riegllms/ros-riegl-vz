# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.9"

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


def _gnssbaseservice_error_decoder(payload):
    return json.loads(payload.decode())
def _gnssbaseservice_statuschanged_decoder(payload):
    return struct.unpack("!B", payload)[0]
def _gnssbaseservice_satelliteschanged_decoder(payload):
    return struct.unpack("!B", payload)[0]
def _gnssbaseservice_lograwdatachanged_decoder(payload):
    return struct.unpack("!?", payload)[0]
def _gnssbaseservice_timesyncchanged_decoder(payload):
    return struct.unpack("!i", payload)[0]
def _gnssbaseservice_positionupdate_decoder(payload):
    return payload.decode()

class GnssBaseService(object):
    """"""

    class GNSSFix(enum.IntEnum):
        GNSS_FIX_INVALID = 0
        GNSS_FIX_SINGLE = 1
        GNSS_FIX_DGPS = 2
        GNSS_FIX_TIME_ONLY = 3
        GNSS_FIX_RTK_FIXED = 4
        GNSS_FIX_RTK_FLOAT = 5
        GNSS_FIX_ESTIMATED = 6
        GNSS_FIX_MANUAL = 7
        GNSS_FIX_SIMULATED = 8

    class ReceiverInterface(enum.IntEnum):
        REC_INT_NONE = -1
        REC_INT_INTERNAL = 0
        REC_INT_SERIAL_ON_TOP = 1
        REC_INT_SERIAL_FROM_BOTTOM = 2
        REC_INT_SERIAL = 3
        REC_INT_UDP_BROADCAST = 4
        REC_INT_TCP_CLIENT = 5
        REC_INT_VZ_ANT = 6
        REC_INT_FILE = 7

    class TimeSyncStatus(enum.IntEnum):
        TIMESYNC_INACTIVE = -1
        TIMESYNC_NO_SYNCHRONIZATION = 0
        TIMESYNC_SYNCHRONIZATION_LOST = 1
        TIMESYNC_SYNCHRONIZATION_WITHIN_LAST_SECOND = 3

    class RTKSupport(enum.IntEnum):
        RTK_SUPPORT_UNKNOWN = -1
        RTK_IS_NOT_SUPPORTED = 0
        RTK_IS_SUPPORTED = 1

    class MasterMode(enum.IntEnum):
        MM_TEGRA = 0
        MM_ZYNQ = 1

    GNSS_FIX_INVALID = GNSSFix.GNSS_FIX_INVALID
    GNSS_FIX_SINGLE = GNSSFix.GNSS_FIX_SINGLE
    GNSS_FIX_DGPS = GNSSFix.GNSS_FIX_DGPS
    GNSS_FIX_TIME_ONLY = GNSSFix.GNSS_FIX_TIME_ONLY
    GNSS_FIX_RTK_FIXED = GNSSFix.GNSS_FIX_RTK_FIXED
    GNSS_FIX_RTK_FLOAT = GNSSFix.GNSS_FIX_RTK_FLOAT
    GNSS_FIX_ESTIMATED = GNSSFix.GNSS_FIX_ESTIMATED
    GNSS_FIX_MANUAL = GNSSFix.GNSS_FIX_MANUAL
    GNSS_FIX_SIMULATED = GNSSFix.GNSS_FIX_SIMULATED
    REC_INT_NONE = ReceiverInterface.REC_INT_NONE
    REC_INT_INTERNAL = ReceiverInterface.REC_INT_INTERNAL
    REC_INT_SERIAL_ON_TOP = ReceiverInterface.REC_INT_SERIAL_ON_TOP
    REC_INT_SERIAL_FROM_BOTTOM = ReceiverInterface.REC_INT_SERIAL_FROM_BOTTOM
    REC_INT_SERIAL = ReceiverInterface.REC_INT_SERIAL
    REC_INT_UDP_BROADCAST = ReceiverInterface.REC_INT_UDP_BROADCAST
    REC_INT_TCP_CLIENT = ReceiverInterface.REC_INT_TCP_CLIENT
    REC_INT_VZ_ANT = ReceiverInterface.REC_INT_VZ_ANT
    REC_INT_FILE = ReceiverInterface.REC_INT_FILE
    TIMESYNC_INACTIVE = TimeSyncStatus.TIMESYNC_INACTIVE
    TIMESYNC_NO_SYNCHRONIZATION = TimeSyncStatus.TIMESYNC_NO_SYNCHRONIZATION
    TIMESYNC_SYNCHRONIZATION_LOST = TimeSyncStatus.TIMESYNC_SYNCHRONIZATION_LOST
    TIMESYNC_SYNCHRONIZATION_WITHIN_LAST_SECOND = TimeSyncStatus.TIMESYNC_SYNCHRONIZATION_WITHIN_LAST_SECOND
    RTK_SUPPORT_UNKNOWN = RTKSupport.RTK_SUPPORT_UNKNOWN
    RTK_IS_NOT_SUPPORTED = RTKSupport.RTK_IS_NOT_SUPPORTED
    RTK_IS_SUPPORTED = RTKSupport.RTK_IS_SUPPORTED
    MM_TEGRA = MasterMode.MM_TEGRA
    MM_ZYNQ = MasterMode.MM_ZYNQ

    def __init__(self, address):
        self._svc = riconnect.Service("GnssBaseService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_gnssbaseservice_error_decoder)
        self._statusChanged = ServiceSignal(self._svc, "statusChanged", decoderFn=_gnssbaseservice_statuschanged_decoder)
        self._satellitesChanged = ServiceSignal(self._svc, "satellitesChanged", decoderFn=_gnssbaseservice_satelliteschanged_decoder)
        self._configChanged = ServiceSignal(self._svc, "configChanged")
        self._logRawDataChanged = ServiceSignal(self._svc, "logRawDataChanged", decoderFn=_gnssbaseservice_lograwdatachanged_decoder)
        self._timesyncChanged = ServiceSignal(self._svc, "timesyncChanged", decoderFn=_gnssbaseservice_timesyncchanged_decoder)
        self._positionUpdate = ServiceSignal(self._svc, "positionUpdate", decoderFn=_gnssbaseservice_positionupdate_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def storDir(self):
        """Return the path to the directory where the raw data logs are stored.

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("storDir_661b6317", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setStorDir(self, value):
        """Set the path of the directory where the raw data logs are stored.

           Arguments:
             value (str): the new target directory"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setStorDir_f4d5e9ce", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)

    def defConfig(self):
        """Return the default configuration.

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("defConfig_304f06aa", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setDefConfig(self, value):
        """Set the default configuration.

           The default configuration is loaded at startup of the service.

           Arguments:
             value (str): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setDefConfig_40617cc7", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)

    def logRawData(self):
        """Return true if the raw GNSS data is written to disk.

           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("logRawData_d52bd544", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setLogRawData(self, value):
        """Set this value to true if the raw GNSS data should be written to disk.

           The target directory where the created data files are stored can be set
           via setStorDir().

           Arguments:
             value (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if value else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setLogRawData_c9f8bb57", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)

    def configDir(self):
        """Return the directory path where the GNSS antenna configurations are stored.

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("configDir_d85bb6d4", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setConfigDir(self, value):
        """Set the directory path where the GNSS antenna configurations are stored.

           Arguments:
             value (str): the new directory path"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setConfigDir_468d3284", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)

    def posValidTimeout(self):
        """Return the position valid timeout in seconds.

           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("posValidTimeout_6c3f7739", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setPosValidTimeout(self, value):
        """Set the position valid timeout in seconds.

           Arguments:
             value (float): the position valid timeout n seconds"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].f = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setPosValidTimeout_bd591915", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)

    def activeConfiguration(self):
        """Return the name of the active configuration.

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("activeConfiguration_47111055", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def listConfigs(self):
        """Return the list of available configurations.

           Returns: list(str)
             list of available configurations"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("listConfigs_c72ea32c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=1)
        outputs = []
        _ot_list = []
        _otsplits = rtransfers[0][0:-1].split(b"\0") if rtransfers[0] else []
        for part in _otsplits:
            _ot_list.append(part.decode())
        outputs.append(_ot_list)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def loadConfig(self, name):
        """Load the specified configuration.

           Arguments:
             name (str): the name of the configuration"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("loadConfig_3244abbc", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)

    def saveConfig(self, filename):
        """Save the current configuration to disk.

           If filename is an absolute path then it will be used as is, otherwise
           the file will be stored in the configuration directory (see setConfigDir()).

           Arguments:
             filename (str): the file name of the created configuration"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = filename
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("saveConfig_a23f5a1e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)

    def getConfig(self, name):
        """Return the configuration with the specified name.

           The data is returned as JSON string in the following format:
           {
             "antPosSOCS": [0, 0, 0.107],
             "antRotating": true,
             "rawDataAvailable": false,
             "recDescription": "",
             "recInterf": 0,
             "recInterfDescr": "com:/dev/ttyACM1:115200",
             "recMessages": ["","","","","","","","","","","","","","","",""],
             "recModel": "LEA-M8T",
             "recName": "Int. GNSS",
             "recOriginCS" : "WGS84, geographic 3D",
             "recOriginCSIdent" : "EPSG::4979",
             "recVendor": "u-blox"
           }

           Arguments:
             name (str): the name of the configuration

           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getConfig_5bb7b387", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setConfig(self, json_str):
        """Set the parameters of the active configuration.

           Arguments:
             json_str (str): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = json_str
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setConfig_ff6cba9e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)

    def estimateInfo(self):
        """Return information about the current position estimate.

           The data is returned as JSON string in the following format:
           {
             "baseline_length": 0,
             "diff_age": null,
             "fix": 1,
             "fix_info": "Single",
             "frame_angle": 187508,
             "hdop": 0.96999996900558472,
             "height": 372.66201782226562,
             "hmsl": 329.76400756835938,
             "hor_acc": 10.771000862121582,
             "latitude": 48.659607399999999,
             "longitude": 15.663604899999999,
             "num_sat": 8,
             "pdop": 1.6100000143051147,
             "pos_acc": 16.270000457763672,
             "recOriginCS" : "WGS84, geographic 3D",
             "recOriginCSIdent" : "EPSG::4979",
             "ref_id" : -1,
             "vdop": 1.2799999713897705,
             "ver_acc": 12.196001052856445
           }

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("estimateInfo_f1cdae4b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def satelliteInfo(self):
        """Return information about the currently visible satellites.

           The data is returned as JSON string in the following format:
           {
             "num_sat": 8
           }

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("satelliteInfo_07c9de1b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def receiverInfo(self):
        """Return information about the receiver.

           The data is returned as JSON string in the following format:
           {
             "recInfo": ["EXT CORE 3.01 (d080e3)","00080000","ROM BASE 2.01 (75331)","FWVER=HPG 1.30REF","PROTVER=20.20","FIS=0xEF4015 (200028)","GPS;GLO;BDS","QZSS"],
                "recModel": "NEO-M8P",
             "recVendor" : "u-blox"
             "rtkSupport" : 1
           }

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("receiverInfo_e2c9f3e4", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def timeInfo(self):
        """Return the current time information.

           The data is returned as JSON string in the following format:
           {
             "gps_time_acc": 9.4999997202194209e-08,
             "gps_week_number": 1887,
             "gps_week_seconds": 292094.99971575098,
             "leap seconds": 17,
             "utc_date_time": "2016-03-09T09:07:58Z",
             "utc_seconds_of_day": 32878,
             "utc_time_acc": 9.4999997202194209e-08
           }

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("timeInfo_1d370502", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def timesyncInfo(self):
        """Return the current time synchronization information.

           The data is returned as JSON string in the following format:
           {
             "exttime": 58258.0,
             "status": 0
           }

           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("timesyncInfo_4c9f2bdd", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
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

    def statusChanged(self):
        """This signal is emitted when the GNSS fix changed.

           The signal payload contains the new GNSS fix.

           Returns: ServiceSignal
           Payload: int"""
        return self._statusChanged

    def satellitesChanged(self):
        """This signal is emitted when the number of visible satellites changed.

           The signal payload contains the new number of visible satellites.

           Returns: ServiceSignal
           Payload: int"""
        return self._satellitesChanged

    def configChanged(self):
        """This signal is emitted when the active configuration changed.

           Returns: ServiceSignal"""
        return self._configChanged

    def logRawDataChanged(self):
        """This signal is emitted when the log raw data setting changed.

           Returns: ServiceSignal
           Payload: bool"""
        return self._logRawDataChanged

    def timesyncChanged(self):
        """This signal is emitted when the time synchronization status changed.

           The signal payload contains the new time synchronization status.

           Returns: ServiceSignal
           Payload: int"""
        return self._timesyncChanged

    def positionUpdate(self):
        """This signal is emitted when new position information is available.

           The signal payload contains the new position information.

           Returns: ServiceSignal
           Payload: str"""
        return self._positionUpdate
