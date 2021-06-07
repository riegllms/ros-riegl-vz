# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.8"

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


def _dataprocservice_error_decoder(payload):
    return json.loads(payload.decode())
def _dataprocservice_finished_decoder(payload):
    return struct.unpack("!B", payload)[0]

class DataprocService(object):
    """The data processing service.
       
       The data processing service is responsible for collecting the measurement data
       created by the scanner and to write the data into RXP files on the system."""

    class SystemError(enum.IntEnum):
        FILE_IO_ERROR = 0
        NO_SPACE_LEFT = 1
        SCANNER_SERVICE_COM_ERROR = 2

    FILE_IO_ERROR = SystemError.FILE_IO_ERROR
    NO_SPACE_LEFT = SystemError.NO_SPACE_LEFT
    SCANNER_SERVICE_COM_ERROR = SystemError.SCANNER_SERVICE_COM_ERROR

    def __init__(self, address):
        self._svc = riconnect.Service("DataprocService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_dataprocservice_error_decoder)
        self._started = ServiceSignal(self._svc, "started")
        self._stopped = ServiceSignal(self._svc, "stopped")
        self._aborted = ServiceSignal(self._svc, "aborted")
        self._finished = ServiceSignal(self._svc, "finished", decoderFn=_dataprocservice_finished_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def rootDir(self):
        """Return the internal target directory for storing data.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("rootDir_0de7714e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setRootDir(self, root_dir):
        """Set the internal target directory used to store data files.
           
           Arguments:
             root_dir (str): the new target directory"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = root_dir
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setRootDir_ce1e3b10", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def rxpSplit(self):
        """Return true if rxp-file split mode is active.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("rxpSplit_7ae73f2d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setRxpSplit(self, enable):
        """Set the rxp-file split mode. Split mode is especially used with endless scans, to automatically create a rxp-file for each scan frame.
           
           Arguments:
             enable (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setRxpSplit_e5b90b23", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def rxpToggle(self):
        """Return true if rxp-file toggle mode is active.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("rxpToggle_2dce166d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setRxpToggle(self, enable):
        """Set the rxp-file toggle mode. Toggle mode is used to automatically disconnect an open current-stream connection with a new connection request, without loosing measurement data.
           
           Arguments:
             enable (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setRxpToggle_eca4d09b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def storeMeasStream(self):
        """Return true if the measurement data stream gets written to disk.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("storeMeasStream_75372fdb", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setStoreMeasStream(self, store):
        """Set to true to write the measurement data stream to disk.
           
           Arguments:
             store (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if store else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setStoreMeasStream_db3fc22b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def storeMonStream(self):
        """Return true if the monitor data stream gets written to disk.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("storeMonStream_c079eb75", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setStoreMonStream(self, store):
        """Set to true to write the monitor data stream to disk.
           
           Arguments:
             store (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if store else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setStoreMonStream_5620b35f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def actualFile(self, stream):
        """Return the file path of the specified measurement stream.
           
           The data stream type can be one of the following:
               0 ... measurement stream
               1 ... monitoring stream
               2 ... housekeeping stream
               3 ... alert stream
           
           Arguments:
             stream (int): the data stream type
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = stream
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("actualFile_b7806cc7", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def start(self, dir, fbasename):
        """Start measurement data acquisition.
           
           This method is non-blocking. Use the method isRunning(), or the signals
           'started', 'stopped', 'aborted' or 'finished' to get more information
           about the acquisition progress.
           
           The created RXP files will be called basename.rxp and basename.mon.rxp
           where basename will be replaced with the specified value.
           
           Arguments:
             dir (str): The public directory for storing file data.
             fbasename (str): 
           
           Returns: int
             0 if successfull, -1 if file open failed."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = dir
        inputs[1].s = fbasename
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("start_c2dc7b7e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].i32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def abort(self):
        """Abort the current measurement data acquisition.
           
           Calling this function may cause corrupted file data at the end of the files.
           This function should only be called if stop() has no effect.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("abort_5bb94a1c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].i32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def stop(self):
        """Stop the current measurement data acquisition.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("stop_ef399b2d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].i32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def isRunning(self):
        """Return true if the data acquisition is in progress.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isRunning_39044c41", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
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

    def started(self):
        """This signal is emitten when the data acquisition started.
           
           Returns: ServiceSignal"""
        return self._started

    def stopped(self):
        """This signal is emitted when the data acquisition was stopped.
           
           Returns: ServiceSignal"""
        return self._stopped

    def aborted(self):
        """This signal is emitted when the data acquisition was aborted.
           
           Returns: ServiceSignal"""
        return self._aborted

    def finished(self):
        """This signal is emitted when the data acquisition finished.
           
           The signal payload contains information about the success of failure of
           the data acquisition. It can be one of the following values:
               0 = success
               1 = acquisition was canceled
               2 = acquisition was aborted
               3 = acquisition failed due to some error
           
           Returns: ServiceSignal
           Payload: int"""
        return self._finished
