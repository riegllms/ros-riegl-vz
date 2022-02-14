# pylint: skip-file
import threading
import enum
import weakref
import riconnect

__version__ = "1.3"

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
    def __init__(self):
        self._connections = []
        self._lock = threading.Lock()
    def connect(self, fn):
        """Connect the specified callback function to the signal."""
        sc = ServiceSignalConnection(self, fn)
        with self._lock:
            self._connections.append(sc)
        return sc
    def _disconnect(self, sc):
        with self._lock:
            if sc in self._connections:
                self._connections.remove(sc)
                sc._connected = False
    def emit(self, payload=None):
        with self._lock:
            try:
                for sc in self._connections:
                    fn = sc._fn() if isinstance(sc._fn, weakref.WeakMethod) else sc._fn
                    if fn is not None:
                        if payload is not None:
                            fn(payload)
                        else:
                            fn()
            except Exception as err:
                print(err)

class InstrumentInfo(dict):
    def __init__(self, *args, **kwargs):
        self['identifier'] = None
        self['serialNumber'] = None
        self['modelNumber'] = None
        self['modifier'] = None
        super().__init__(*args, **kwargs)
    @property
    def identifier(self):
        return self['identifier']
    @identifier.setter
    def identifier(self, value):
        self['identifier'] = value
    @property
    def serialNumber(self):
        return self['serialNumber']
    @serialNumber.setter
    def serialNumber(self, value):
        self['serialNumber'] = value
    @property
    def modelNumber(self):
        return self['modelNumber']
    @modelNumber.setter
    def modelNumber(self, value):
        self['modelNumber'] = value
    @property
    def modifier(self):
        return self['modifier']
    @modifier.setter
    def modifier(self, value):
        self['modifier'] = value

class MeasurementProgram(dict):
    def __init__(self, *args, **kwargs):
        self['id'] = None
        self['name'] = None
        self['prr'] = None
        super().__init__(*args, **kwargs)
    @property
    def id(self):
        return self['id']
    @id.setter
    def id(self, value):
        self['id'] = value
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def prr(self):
        return self['prr']
    @prr.setter
    def prr(self, value):
        self['prr'] = value

class FineScanPattern(dict):
    def __init__(self, *args, **kwargs):
        self['theta'] = None
        self['phi'] = None
        self['range'] = None
        self['reflectance'] = None
        self['targetDiameter'] = None
        self['overlap'] = None
        self['oversize'] = None
        super().__init__(*args, **kwargs)
    @property
    def theta(self):
        return self['theta']
    @theta.setter
    def theta(self, value):
        self['theta'] = value
    @property
    def phi(self):
        return self['phi']
    @phi.setter
    def phi(self, value):
        self['phi'] = value
    @property
    def range(self):
        return self['range']
    @range.setter
    def range(self, value):
        self['range'] = value
    @property
    def reflectance(self):
        return self['reflectance']
    @reflectance.setter
    def reflectance(self, value):
        self['reflectance'] = value
    @property
    def targetDiameter(self):
        return self['targetDiameter']
    @targetDiameter.setter
    def targetDiameter(self, value):
        self['targetDiameter'] = value
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

class LineScanPattern(dict):
    def __init__(self, *args, **kwargs):
        self['thetaStart'] = None
        self['thetaStop'] = None
        self['thetaIncrement'] = None
        self['phi'] = None
        super().__init__(*args, **kwargs)
    @property
    def thetaStart(self):
        return self['thetaStart']
    @thetaStart.setter
    def thetaStart(self, value):
        self['thetaStart'] = value
    @property
    def thetaStop(self):
        return self['thetaStop']
    @thetaStop.setter
    def thetaStop(self, value):
        self['thetaStop'] = value
    @property
    def thetaIncrement(self):
        return self['thetaIncrement']
    @thetaIncrement.setter
    def thetaIncrement(self, value):
        self['thetaIncrement'] = value
    @property
    def phi(self):
        return self['phi']
    @phi.setter
    def phi(self, value):
        self['phi'] = value

class RectScanPattern(dict):
    def __init__(self, *args, **kwargs):
        self['thetaStart'] = None
        self['thetaStop'] = None
        self['thetaIncrement'] = None
        self['phiStart'] = None
        self['phiStop'] = None
        self['phiIncrement'] = None
        super().__init__(*args, **kwargs)
    @property
    def thetaStart(self):
        return self['thetaStart']
    @thetaStart.setter
    def thetaStart(self, value):
        self['thetaStart'] = value
    @property
    def thetaStop(self):
        return self['thetaStop']
    @thetaStop.setter
    def thetaStop(self, value):
        self['thetaStop'] = value
    @property
    def thetaIncrement(self):
        return self['thetaIncrement']
    @thetaIncrement.setter
    def thetaIncrement(self, value):
        self['thetaIncrement'] = value
    @property
    def phiStart(self):
        return self['phiStart']
    @phiStart.setter
    def phiStart(self, value):
        self['phiStart'] = value
    @property
    def phiStop(self):
        return self['phiStop']
    @phiStop.setter
    def phiStop(self, value):
        self['phiStop'] = value
    @property
    def phiIncrement(self):
        return self['phiIncrement']
    @phiIncrement.setter
    def phiIncrement(self, value):
        self['phiIncrement'] = value

class ScannerService(object):
    """"""

    class ScanMode(enum.IntEnum):
        SCN_MODE_NONE = 0
        SCN_MODE_RECTANGULAR_FOV = 1
        SCN_MODE_FINE_SCAN = 3
        SCN_MODE_LINE_SCAN = 4

    class ParkMode(enum.IntEnum):
        SCN_PARK_MODE_DISABLED = 0
        SCN_PARK_MODE_AUTOMATIC = 1
        SCN_PARK_MODE_LINE_ONLY = 2
        SCN_PARK_MODE_FRAME_ONLY = 3

    class ScanDirection(enum.IntEnum):
        SCN_DIR_MODE_AUTOMATIC = 0
        SCN_DIR_MODE_FORCED_POSITIVE_UPSCAN = 1
        SCN_DIR_MODE_FORCED_NEGATIVE_DOWNSCAN = 2

    class MeasurementResultState(enum.IntEnum):
        MEAS_SUCCESS = 0
        MEAS_STOP = 1
        MEAS_ABORT = 2

    class AcquisitionMode(enum.IntEnum):
        MEAS_RANGE_ONLY = 0
        MEAS_WAVEFORM_ONLY = 1
        MEAS_RANGE_WAVEFORM = 2

    SCN_MODE_NONE = ScanMode.SCN_MODE_NONE
    SCN_MODE_RECTANGULAR_FOV = ScanMode.SCN_MODE_RECTANGULAR_FOV
    SCN_MODE_FINE_SCAN = ScanMode.SCN_MODE_FINE_SCAN
    SCN_MODE_LINE_SCAN = ScanMode.SCN_MODE_LINE_SCAN
    SCN_PARK_MODE_DISABLED = ParkMode.SCN_PARK_MODE_DISABLED
    SCN_PARK_MODE_AUTOMATIC = ParkMode.SCN_PARK_MODE_AUTOMATIC
    SCN_PARK_MODE_LINE_ONLY = ParkMode.SCN_PARK_MODE_LINE_ONLY
    SCN_PARK_MODE_FRAME_ONLY = ParkMode.SCN_PARK_MODE_FRAME_ONLY
    SCN_DIR_MODE_AUTOMATIC = ScanDirection.SCN_DIR_MODE_AUTOMATIC
    SCN_DIR_MODE_FORCED_POSITIVE_UPSCAN = ScanDirection.SCN_DIR_MODE_FORCED_POSITIVE_UPSCAN
    SCN_DIR_MODE_FORCED_NEGATIVE_DOWNSCAN = ScanDirection.SCN_DIR_MODE_FORCED_NEGATIVE_DOWNSCAN
    MEAS_SUCCESS = MeasurementResultState.MEAS_SUCCESS
    MEAS_STOP = MeasurementResultState.MEAS_STOP
    MEAS_ABORT = MeasurementResultState.MEAS_ABORT
    MEAS_RANGE_ONLY = AcquisitionMode.MEAS_RANGE_ONLY
    MEAS_WAVEFORM_ONLY = AcquisitionMode.MEAS_WAVEFORM_ONLY
    MEAS_RANGE_WAVEFORM = AcquisitionMode.MEAS_RANGE_WAVEFORM

    def __init__(self, address):
        self._svc = riconnect.Service("SCANNER")
        self._svc.open(address)
        self._lock = threading.Lock()

        # connect to signals
        self._svc.subscribe("IPCIRQ_IRQ_MEAS_ACQ_START", weakref.WeakMethod(self._onMeasAcqStart))
        self._svc.subscribe("IPCIRQ_IRQ_MEAS_ACQ_STOP", weakref.WeakMethod(self._onMeasAcqStop))
        self._svc.subscribe("IPCIRQ_IRQ_MEAS_ACQ_DONE", weakref.WeakMethod(self._onMeasAcqDone))
        self._svc.subscribe("IPCIRQ_IRQ_MEAS_ACQ_ABORT", weakref.WeakMethod(self._onMeasAcqAbort))

        # service signals
        self._measurementStarted = ServiceSignal()
        self._measurementFinished = ServiceSignal()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def _onMeasAcqStart(self, payload):
        self._measurementStarted.emit()

    def _onMeasAcqStop(self, payload):
        self._measurementFinished.emit(ScannerService.MEAS_STOP)

    def _onMeasAcqDone(self, payload):
        self._measurementFinished.emit(ScannerService.MEAS_SUCCESS)

    def _onMeasAcqAbort(self, payload):
        self._measurementFinished.emit(ScannerService.MEAS_ABORT)

    def instrumentInformation(self):
        """Return the instrument information."""
        with self._lock:
            r = InstrumentInfo()
            r.identifier = self._svc.getProperty("INST_IDENT").s
            r.serialNumber = self._svc.getProperty("SN").s
            r.modelNumber = self._svc.getProperty("INST_MODEL_NUM").s
            r.modifier = self._svc.getProperty("INST_MODIF").s
            return r

    def isBusy(self, blocking):
        """Query measurement state, either busy or not

           Query measurement state, either busy or not. Busy indicates that a
           measurement is in progress or a measurement has finished but not all data
           has been fetched.

           Arguments:
             blocking (bool): if true wait as long as measurement is busy"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if blocking else 0
        with self._lock:
            rvalues = self._svc.callFunction("MEAS_BUSY", inputs=inputs)[0]
        return True if rvalues[0].i32 > 0 else False

    def startMeasurement(self):
        """Start measurement.

           Starts the measurement and scan motion. The result data can be read from
           the data port."""
        with self._lock:
            self._svc.callFunction("MEAS_START")

    def stopMeasurement(self):
        """Stop measurement.

           If no measurement is in progress, then this operation does nothing. Note
           that after stopping the measurement, data that has already been acquired,
           but not fetched via the data port must be fetched by the reading
           application. While data is still in the buffer, calling isBusy() will
           return true. To dismiss all data already acquired but not yet fetched use
           abortMeasurement()."""
        with self._lock:
            self._svc.callFunction("MEAS_STOP")

    def abortMeasurement(self):
        """Abort measurement.

           If no measurement is in progress, then this operation does nothing. As
           opposed to stopMeasurement() this method dismisses all data which has
           already been acquired but not yet fetched from the data port."""
        with self._lock:
            self._svc.callFunction("MEAS_ABORT")

    def measurementPrograms(self):
        """Returns a list of the available measurement programs.

           Different measurement programs have different operating parameters
           (e.g. measurement rate or laser pulse rate)."""
        with self._lock:
            # MEAS_PROG_LIMIT ist the last available index
            numProgs = self._svc.getProperty("MEAS_PROG_LIMIT").u32 + 1
            r = []
            for idx in range(0, numProgs):
                prog = MeasurementProgram()
                prog.id = idx
                prog.name = self._svc.getProperty("MEAS_PROG_NAME", idx).s
                prog.prr = self._svc.getProperty("LASER_PRR_PROG", idx).f
                r.append(prog)
            return r

    def activeMeasurementProgram(self):
        """Return the ID of the active measurement program."""
        with self._lock:
            return self._svc.getProperty("MEAS_PROG").i32

    def setMeasurementProgram(self, progId):
        """Set the active measurement program.

           Arguments:
             progId (int): the program ID"""
        if not isinstance(progId, int):
            raise TypeError("Program ID must be and int.")
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = progId
        with self._lock:
            self._svc.callFunction("MEAS_SET_PROG", inputs=inputs)

    def setFineScan(self, pattern):
        """Set (apply) a fine scan pattern.

           This is one of the central methods for defining a scan pattern.
           A fine scan is typically used to scan small, highly reflective reference
           targets (reflectors). Once the position of such a refelctor is roughly found
           in a scan with medium or low resolution, the fine scan with high resolution
           allows accurate detection of the reflector object's position. The FineScan
           is a special form of Rectangular FOV Scan. If the requested scan pattern
           contradict the scanner's range and speed limits then the values are modified.

           After the pattern is set, measurement and scan motion have to be started
           using startMeasurement().

           Arguments:
             pattern (FineScanPattern): the new scan pattern"""
        if not isinstance(pattern, FineScanPattern):
            raise TypeError("Pattern must be of type FineScanPattern.")
        inputs = [riconnect.Value() for i in range(0, 7)]
        inputs[0].f = pattern.theta
        inputs[1].f = pattern.phi
        inputs[2].f = pattern.range
        inputs[3].f = pattern.reflectance
        inputs[4].f = pattern.targetDiameter
        inputs[5].f = pattern.overlap
        inputs[6].f = pattern.oversize
        with self._lock:
            self._svc.callFunction("SCN_SET_FINE_SCAN", inputs=inputs)

    def lastSetFineScan(self):
        """Return the last fine scan pattern set via setFineScan()."""
        r = FineScanPattern()
        with self._lock:
            r.theta = self._svc.getProperty("SCN_SET_FS_THETA_SOCS").f
            r.phi = self._svc.getProperty("SCN_SET_FS_PHI_SOCS").f
            r.range = self._svc.getProperty("SCN_SET_FS_RANGE").f
            r.reflectance = self._svc.getProperty("SCN_SET_FS_REFLECTANCE").f
            r.targetDiameter = self._svc.getProperty("SCN_SET_FS_DIAMETER").f
            r.overlap = self._svc.getProperty("SCN_SET_FS_OVERLAP").f
            r.oversize = self._svc.getProperty("SCN_SET_FS_OVERSIZE").f
        return r

    def setLineScan(self, pattern):
        """Set (apply) a line scan pattern.

           This is one of the central methods for defining a scan pattern.
           A line scan is a 2D scan. There is no frame movement during scanning, only
           at scan startup does the scanner align itself to the desired frame
           direction. If the requested scan pattern contradicts the scanner's range
           and speed limits then the values are modified.

           After the pattern is set, measurement and scan motion have to be started
           using startMeasurement(). Note that the line scan is an unlimited scan,
           meaning that it continues until stopMeasurement() is called.

           Arguments:
             pattern (LineScanPattern): the new scan pattern"""
        if not isinstance(pattern, LineScanPattern):
            raise TypeError("Pattern must be of type LineScanPattern.")
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].f = pattern.thetaStart
        inputs[1].f = pattern.thetaStop
        inputs[2].f = pattern.thetaIncrement
        inputs[3].f = pattern.phi
        with self._lock:
            self._svc.callFunction("SCN_SET_LINE_SCAN", inputs=inputs)

    def lastSetLineScan(self):
        """Return the last line scan pattern set via setLineScan()."""
        r = LineScanPattern()
        with self._lock:
            r.thetaStart = self._svc.getProperty("SCN_SET_THETA_SOCS_START").f
            r.thetaStop = self._svc.getProperty("SCN_SET_THETA_SOCS_STOP").f
            r.thetaIncrement = self._svc.getProperty("SCN_SET_THETA_SOCS_INCR").f
            r.phi = self._svc.getProperty("SCN_SET_PHI_SOCS_START").f
        return r

    def setRectScan(self, pattern, numIterations):
        """Set (apply) a Rectangular Field Of View (FOV) scan pattern.

           This is one of the central methods for defining a scan pattern.
           If the requested scan pattern contradicts the scanner's range and speed
           limits then the values are modified. If the number of iterations
           (successive scans) is zero then the pattern is scanned until
           stopMeasurement() is called.

           After the pattern is set, measurement and scan motion have to be started
           using startMeasurement().

           Arguments:
             pattern (RectScanPattern): the new scan pattern
             numIterations (int): the number of times the scanpattern should be scanned"""
        if not isinstance(pattern, RectScanPattern):
            raise TypeError("Pattern must be of type RectScanPattern.")
        if not isinstance(numIterations, int):
            raise TypeError("NumIterations must be an int.")
        if numIterations < 0:
            raise ValueError("NumIterations must be positive.")
        inputs = [riconnect.Value() for i in range(0, 7)]
        inputs[0].f = pattern.thetaStart
        inputs[1].f = pattern.thetaStop
        inputs[2].f = pattern.thetaIncrement
        inputs[3].f = pattern.phiStart
        inputs[4].f = pattern.phiStop
        inputs[5].f = pattern.phiIncrement
        inputs[6].u32 = numIterations
        with self._lock:
            self._svc.callFunction("SCN_SET_RECT_FOV_SEQ", inputs=inputs)

    def lastSetRectScan(self):
        """Return the last scan pattern set via setRectScan()."""
        r = RectScanPattern()
        with self._lock:
            r.thetaStart = self._svc.getProperty("SCN_SET_THETA_SOCS_START").f
            r.thetaStop = self._svc.getProperty("SCN_SET_THETA_SOCS_STOP").f
            r.thetaIncrement = self._svc.getProperty("SCN_SET_THETA_SOCS_INCR").f
            r.phiStart = self._svc.getProperty("SCN_SET_PHI_SOCS_START").f
            r.phiStop = self._svc.getProperty("SCN_SET_PHI_SOCS_STOP").f
            r.phiIncrement = self._svc.getProperty("SCN_SET_PHI_SOCS_INCR").f
        return r

    def activeScanMode(self):
        """Return the active scan mode."""
        with self._lock:
            return ScannerService.ScanMode(self._svc.getProperty("SCN_MODE").i32)

    def activeScanPattern(self):
        """Return the active scan pattern."""
        with self._lock:
            rvalues = self._svc.callFunction("FOV")[0]
        r = RectScanPattern()
        r.thetaStart = rvalues[1].f
        r.thetaStop = rvalues[2].f
        r.thetaIncrement = rvalues[3].f
        r.phiStart = rvalues[4].f
        r.phiStop = rvalues[5].f
        r.phiIncrement = rvalues[6].f
        return r

    def numScanIterations(self):
        """Return the number of times the rectangular scan pattern is scanned.

           A value of zero means that the rectangular scan pattern is scanned until
           stopMeasurement() is called."""
        with self._lock:
            return self._svc.getProperty("SCN_NUM_SCANS").u32

    def scanTime(self):
        """Return the scan time of the active scan pattern (in seconds).

           If the active scan pattern is a 'line scan' then 0 is returned. For
           rectangular scan patterns, the returned time represents the scan time
           for one scan, not the total time of all scan iterations.

           totalTime = scanTime() * numScanIterations()"""
        with self._lock:
            return self._svc.getProperty("SCN_TCALC").u32

    def scanTimeElapsed(self):
        """Return the elapsed time since the measurement was started (in seconds).

           If the active scan pattern is a 'line scan' then 0 is returned. For
           rectangular scan patterns, the returned time represents the elapsed scan
           time for one scan, not the total time since startMeasurement() was called."""
        with self._lock:
            return self._svc.getProperty("SCN_TELAPS").u32

    def parkMode(self):
        """Return the scanner park mode."""
        with self._lock:
            return ScannerService.ParkMode(self._svc.getProperty("SCN_PARK_MODE").i32)

    def setParkMode(self, mode):
        """Set the scanner park mode.

           When set to AUTOMATIC, the scanner returns to its parking position after
           every completed scan. If a scan is stopped before it was completed then
           the scanner does not return to the parking position.

           Arguments:
             mode (ParkMode): the new park mode"""
        value = riconnect.Value()
        value.i32 = int(mode)
        with self._lock:
            self._svc.setProperty("SCN_PARK_MODE", value)

    def parkPosition(self):
        """Return the scanner park position (in degrees)."""
        with self._lock:
            return self._svc.getProperty("SCN_PHI_SOCS_PARK").f

    def setParkPosition(self, phi):
        """Set scanner park position (in degrees).

           Arguments:
             phi (float): the new park position"""
        value = riconnect.Value()
        value.f = float(phi)
        with self._lock:
            self._svc.setProperty("SCN_PHI_SOCS_PARK", value)

    def parkScanner(self):
        """Move scanner to park position."""
        with self._lock:
            self._svc.callFunction("SCN_PARK", timeout=60)

    def alignScanner(self, phi):
        """Move the scanner to the specified position.

           Arguments:
             phi (float): the new frame angle (in degrees)"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].f = phi
        inputs[1].i32 = 1
        with self._lock:
            self._svc.callFunction("SCN_ALIGN_PHI", inputs=inputs, timeout=60)

    def frameAngle(self):
        """Return the scanner's current frame angle (in degrees)."""
        with self._lock:
            return self._svc.getProperty("SCN_PHI_SOCS_THETA90").f

    def acquisitionMode(self):
        """Return data acquisition mode."""
        with self._lock:
            return ScannerService.AcquisitionMode(self._svc.getProperty("MEAS_OUTPUT_TYPE").i32)

    def setAcquisitionMode(self, mode):
        """Set data acquisition mode.

           Arguments:
             mode (AcquisitionMode): new acquisition mode"""
        value = riconnect.Value()
        value.i32 = int(mode)
        with self._lock:
            self._svc.setProperty("MEAS_OUTPUT_TYPE", value)

    def pulseFitDeviationThreshold(self):
        """Return threshold of pulse fit deviation used for full waveform data output."""
        with self._lock:
            return self._svc.getProperty("FILT_WFM_DEV_LIMIT").u32

    def setPulseFitDeviationThreshold(self, value):
        """Set threshold of pulse fit deviation used for full waveform data output.

           This threshold effects the output of full waveform data (MEAS_RANGE_WAVEFORM).
           Full waveform data is only generated for echoes whose pulse shape deviation is
           smaller than the specified threshold. Setting this value to 0 will output
           all waveforms.

           Arguments:
             value (int): new deviation threshold. Must be between 0 and 65535."""
        if value < 0 or value > 65535:
            return ValueError("Deviation threshold out of range.")
        value = riconnect.Value()
        value.u32 = int(value)
        with self._lock:
            self._svc.setProperty("FILT_WFM_DEV_LIMIT", value)

    def measurementStarted(self):
        """This signal is emitted when a measurement was started."""
        return self._measurementStarted

    def measurementFinished(self):
        """This signal is emitted when a measurement has finished."""
        return self._measurementFinished

    def scanDirection(self):
        """Return the scan direction mode.

           New in version 1.2."""
        with self._lock:
            return ScannerService.ScanDirection(self._svc.getProperty("SCN_DIR_MODE").i32)

    def setScanDirection(self, mode):
        """Set the scan direction mode.

           When set to AUTOMATIC, the scanner uses the shortes way to the scan
           pattern start position.

           Arguments:
             mode (ScanDirection): the new scan direction mode

           New in version 1.2."""
        value = riconnect.Value()
        value.i32 = int(mode)
        with self._lock:
            self._svc.setProperty("SCN_DIR_MODE", value)

    def thetaMin(self):
        """Return the minimum theta angle of the scanner's field of view.

           New in version 1.2."""
        with self._lock:
            return self._svc.getProperty("FOV_THETA_MIN").f

    def thetaMax(self):
        """Return the maximum theta angle of the scanner's field of view.

           New in version 1.2."""
        with self._lock:
            return self._svc.getProperty("FOV_THETA_MAX").f

    def calculateScanDuration(self, progId, thetaRange, thetaIncr, phiRange, phiIncr):
        """Calculate the time (in seconds) necessary to aquire a scan with the given
           rectangular scan pattern and measurement program.

           Args:
               progId (int): measurement program ID
               thetaRange (float): theta angel range (thetaStop - thetaStart)
               thetaIncr (float): theta angle resolution
               phiRange (float): phi angel range (phiStop - phiStart)
               phiIncr (float): phi angle resolution

           Returns:
               tuple(seconds, numMeasurements)

           New in version 1.2."""
        inputs = [riconnect.Value() for i in range(0, 5)]
        inputs[0].i32 = progId
        inputs[1].f = thetaRange
        inputs[2].f = thetaIncr
        inputs[3].f = phiRange
        inputs[4].f = phiIncr
        with self._lock:
            rvalues = self._svc.callFunction("SCN_CALC_RECT_FOV", inputs=inputs)[0]
            return (rvalues[0].f, rvalues[1].u32)

    def shutdown(self):
        """Turn the instrument off. It is strongly recommended NOT TO SIMPLY UNPLUG
           the instrument from the power supply but to execute this shutdown method
           or use the on/off power button.

           New in version 1.2."""
        with self._lock:
            self._svc.callFunction("SHUTDOWN")

    def reboot(self):
        """Reboot the instrument.

           New in version 1.2."""
        with self._lock:
            self._svc.callFunction("REBOOT")

    def isMtaEnabled(self):
        """Return True if Multiple Time Around is enabled.

           New in version 1.2."""
        with self._lock:
            return self._svc.getProperty("MEAS_MTA_ENABLE").i32 != 0

    def setMtaEnabled(self, value):
        """Enable or disable Multiple Time Around.

           Args:
               value (bool): True to enable, False to disable MTA
           New in version 1.2."""
        if not isinstance(value, bool):
            raise TypeError("Value must be of type bool.")
        val = riconnect.Value()
        val.i32 = 1 if value else 0
        with self._lock:
            self._svc.setProperty("MEAS_MTA_ENABLE", val)

    def isLaserOn(self):
        """Return laser on state."""
        with self._lock:
            return self._svc.getProperty("LASER").i32 != 0

    def gpsMode(self):
        """Return gps config mode."""
        with self._lock:
            return self._svc.getProperty("GPS_MODE").i32
