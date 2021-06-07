# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.4"

_ridFactory = riconnect.ConnectionFactory()

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


class imu_measurement(dict):
    def __init__(self, *args, **kwargs):
        self['systime'] = None
        self['line_angle'] = None
        self['frame_angle'] = None
        self['gyro_x'] = None
        self['gyro_y'] = None
        self['gyro_z'] = None
        self['acc_x'] = None
        self['acc_y'] = None
        self['acc_z'] = None
        self['mag_x'] = None
        self['mag_y'] = None
        self['mag_z'] = None
        self['temperature'] = None
        self['frame_speed'] = None
        self['flags'] = None
        super().__init__(*args, **kwargs)
    @property
    def systime(self):
        return self['systime']
    @systime.setter
    def systime(self, value):
        self['systime'] = value
    @property
    def line_angle(self):
        return self['line_angle']
    @line_angle.setter
    def line_angle(self, value):
        self['line_angle'] = value
    @property
    def frame_angle(self):
        return self['frame_angle']
    @frame_angle.setter
    def frame_angle(self, value):
        self['frame_angle'] = value
    @property
    def gyro_x(self):
        return self['gyro_x']
    @gyro_x.setter
    def gyro_x(self, value):
        self['gyro_x'] = value
    @property
    def gyro_y(self):
        return self['gyro_y']
    @gyro_y.setter
    def gyro_y(self, value):
        self['gyro_y'] = value
    @property
    def gyro_z(self):
        return self['gyro_z']
    @gyro_z.setter
    def gyro_z(self, value):
        self['gyro_z'] = value
    @property
    def acc_x(self):
        return self['acc_x']
    @acc_x.setter
    def acc_x(self, value):
        self['acc_x'] = value
    @property
    def acc_y(self):
        return self['acc_y']
    @acc_y.setter
    def acc_y(self, value):
        self['acc_y'] = value
    @property
    def acc_z(self):
        return self['acc_z']
    @acc_z.setter
    def acc_z(self, value):
        self['acc_z'] = value
    @property
    def mag_x(self):
        return self['mag_x']
    @mag_x.setter
    def mag_x(self, value):
        self['mag_x'] = value
    @property
    def mag_y(self):
        return self['mag_y']
    @mag_y.setter
    def mag_y(self, value):
        self['mag_y'] = value
    @property
    def mag_z(self):
        return self['mag_z']
    @mag_z.setter
    def mag_z(self, value):
        self['mag_z'] = value
    @property
    def temperature(self):
        return self['temperature']
    @temperature.setter
    def temperature(self, value):
        self['temperature'] = value
    @property
    def frame_speed(self):
        return self['frame_speed']
    @frame_speed.setter
    def frame_speed(self, value):
        self['frame_speed'] = value
    @property
    def flags(self):
        return self['flags']
    @flags.setter
    def flags(self, value):
        self['flags'] = value

class imu_data(dict):
    def __init__(self, *args, **kwargs):
        self['package_index'] = None
        self['package_count'] = None
        self['measurements'] = []
        super().__init__(*args, **kwargs)
        self['measurements'] = [imu_measurement(k) if type(k) == dict else k for k in self.get('measurements', [])]
    @property
    def package_index(self):
        return self['package_index']
    @package_index.setter
    def package_index(self, value):
        self['package_index'] = value
    @property
    def package_count(self):
        return self['package_count']
    @package_count.setter
    def package_count(self, value):
        self['package_count'] = value
    @property
    def measurements(self):
        return self['measurements']
    @measurements.setter
    def measurements(self, value):
        self['measurements'] = value

class gps_pps_data(dict):
    def __init__(self, *args, **kwargs):
        self['local_time'] = None
        self['systime'] = None
        self['pps'] = None
        self['year'] = None
        self['month'] = None
        self['day'] = None
        self['hour'] = None
        self['minute'] = None
        self['second'] = None
        self['flags'] = None
        self['pulse_counter'] = None
        super().__init__(*args, **kwargs)
    @property
    def local_time(self):
        return self['local_time']
    @local_time.setter
    def local_time(self, value):
        self['local_time'] = value
    @property
    def systime(self):
        return self['systime']
    @systime.setter
    def systime(self, value):
        self['systime'] = value
    @property
    def pps(self):
        return self['pps']
    @pps.setter
    def pps(self, value):
        self['pps'] = value
    @property
    def year(self):
        return self['year']
    @year.setter
    def year(self, value):
        self['year'] = value
    @property
    def month(self):
        return self['month']
    @month.setter
    def month(self, value):
        self['month'] = value
    @property
    def day(self):
        return self['day']
    @day.setter
    def day(self, value):
        self['day'] = value
    @property
    def hour(self):
        return self['hour']
    @hour.setter
    def hour(self, value):
        self['hour'] = value
    @property
    def minute(self):
        return self['minute']
    @minute.setter
    def minute(self, value):
        self['minute'] = value
    @property
    def second(self):
        return self['second']
    @second.setter
    def second(self, value):
        self['second'] = value
    @property
    def flags(self):
        return self['flags']
    @flags.setter
    def flags(self, value):
        self['flags'] = value
    @property
    def pulse_counter(self):
        return self['pulse_counter']
    @pulse_counter.setter
    def pulse_counter(self, value):
        self['pulse_counter'] = value

class ImuDataDecoder:
    def decode(self, payload):
        if self._isJsonEncoded(payload):
            return self._decodeJson(payload)
        else:
            return self._decodeRaw(payload)

    def _isJsonEncoded(self, payload):
        if len(payload) > 25 and payload[0] == b'{':
            substr = payload[:25]
            keyFound = False
            for key in [b'measurements', b'package_index', b'package_count']:
                if key in substr:
                    keyFound = True
                    break
            return keyFound
        return False

    def _decodeJson(self, payload):
        return imu_data(json.loads(payload.decode()))

    def _decodeRaw(self, payload):
        if len(payload) < 16:
            raise RuntimeError("Parsing of RAW signal payload failed. Invalid data length.")

        package_id, package_endianess = struct.unpack_from("<2B", payload, 0)
        endian = "<" if package_endianess == 0 else ">"
        package_addon = struct.unpack_from(endian+"2B", payload, 2)
        package_header_size, package_header_cnt = struct.unpack_from(endian+"2H", payload, 4)

        r = imu_data()
        # read package header
        offset = 8
        pkg_index, pkg_count, num_meas = struct.unpack_from(endian+"3i", payload, offset)
        r.package_index = pkg_index
        r.package_count = pkg_count
        offset += package_header_size * package_header_cnt

        pkg_meas_size, pkg_meas_count = struct.unpack_from(endian+"2H", payload, offset)
        offset += 4

        for i in range(pkg_meas_count):
            data = struct.unpack_from(endian+"I2i10hiI", payload, offset)
            meas = imu_measurement()
            meas.systime = data[0]
            meas.line_angle = data[1]
            meas.frame_angle = data[2]
            meas.gyro_x = data[3]
            meas.gyro_y = data[4]
            meas.gyro_z = data[5]
            meas.acc_x = data[6]
            meas.acc_y = data[7]
            meas.acc_z = data[8]
            meas.mag_x = data[9]
            meas.mag_y = data[10]
            meas.mag_z = data[11]
            meas.temperature = data[12]
            meas.frame_speed = data[13]
            meas.flags = data[14]
            r.measurements.append(meas)
            offset += pkg_meas_size
        return r

def _scannereventsservice_error_decoder(payload):
    return json.loads(payload.decode())
def _scannereventsservice_ipcirq_irq_imu_data_decoder(payload):
    return ImuDataDecoder().decode(payload)
def _scannereventsservice_ipcirq_irq_gps_pps_data_decoder(payload):
    return gps_pps_data(json.loads(payload.decode()))

class ScannereventsService(object):
    """"""

    def __init__(self, address):
        self._svc = riconnect.Service("SCANNER", connectionFactory=_ridFactory)
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_scannereventsservice_error_decoder)
        self._IPCIRQ_IRQ_SCAN_START = ServiceSignal(self._svc, "IPCIRQ_IRQ_SCAN_START")
        self._IPCIRQ_IRQ_SCAN_DONE = ServiceSignal(self._svc, "IPCIRQ_IRQ_SCAN_DONE")
        self._IPCIRQ_IRQ_MEAS_READY = ServiceSignal(self._svc, "IPCIRQ_IRQ_MEAS_READY")
        self._IPCIRQ_IRQ_MEAS_ACQ_START = ServiceSignal(self._svc, "IPCIRQ_IRQ_MEAS_ACQ_START")
        self._IPCIRQ_IRQ_MEAS_ACQ_STOP = ServiceSignal(self._svc, "IPCIRQ_IRQ_MEAS_ACQ_STOP")
        self._IPCIRQ_IRQ_MEAS_ACQ_DONE = ServiceSignal(self._svc, "IPCIRQ_IRQ_MEAS_ACQ_DONE")
        self._IPCIRQ_IRQ_MEAS_ACQ_ABORT = ServiceSignal(self._svc, "IPCIRQ_IRQ_MEAS_ACQ_ABORT")
        self._IPCIRQ_IRQ_PROPERTY_UPDATE = ServiceSignal(self._svc, "IPCIRQ_IRQ_PROPERTY_UPDATE")
        self._IPCIRQ_IRQ_ERROR = ServiceSignal(self._svc, "IPCIRQ_IRQ_ERROR")
        self._IPCIRQ_IRQ_ERROR_PUSH = ServiceSignal(self._svc, "IPCIRQ_IRQ_ERROR_PUSH")
        self._IPCIRQ_IRQ_ERROR_ACK = ServiceSignal(self._svc, "IPCIRQ_IRQ_ERROR_ACK")
        self._IPCIRQ_IRQ_SEQUENCE = ServiceSignal(self._svc, "IPCIRQ_IRQ_SEQUENCE")
        self._IPCIRQ_IRQ_SEQUENCE_DONE_OK = ServiceSignal(self._svc, "IPCIRQ_IRQ_SEQUENCE_DONE_OK")
        self._IPCIRQ_IRQ_SEQUENCE_FAILED = ServiceSignal(self._svc, "IPCIRQ_IRQ_SEQUENCE_FAILED")
        self._IPCIRQ_IRQ_IMG_ACQ_START = ServiceSignal(self._svc, "IPCIRQ_IRQ_IMG_ACQ_START")
        self._IPCIRQ_IRQ_IMG_ACQ_STOP = ServiceSignal(self._svc, "IPCIRQ_IRQ_IMG_ACQ_STOP")
        self._IPCIRQ_IRQ_IMG_ACQ_DONE = ServiceSignal(self._svc, "IPCIRQ_IRQ_IMG_ACQ_DONE")
        self._IPCIRQ_IRQ_IMG_UPDATE = ServiceSignal(self._svc, "IPCIRQ_IRQ_IMG_UPDATE")
        self._IPCIRQ_IRQ_IMG_EXPOSE = ServiceSignal(self._svc, "IPCIRQ_IRQ_IMG_EXPOSE")
        self._IPCIRQ_IRQ_IMG_ACQ_CMD = ServiceSignal(self._svc, "IPCIRQ_IRQ_IMG_ACQ_CMD")
        self._IPCIRQ_IRQ_SHUTDOWN_REQ = ServiceSignal(self._svc, "IPCIRQ_IRQ_SHUTDOWN_REQ")
        self._IPCIRQ_IRQ_FRAME_UPSCAN = ServiceSignal(self._svc, "IPCIRQ_IRQ_FRAME_UPSCAN")
        self._IPCIRQ_IRQ_FRAME_DOWNSCAN = ServiceSignal(self._svc, "IPCIRQ_IRQ_FRAME_DOWNSCAN")
        self._IPCIRQ_IRQ_CAMERA_MOUNT_DETECTION = ServiceSignal(self._svc, "IPCIRQ_IRQ_CAMERA_MOUNT_DETECTION")
        self._IPCIRQ_IRQ_IMU_DATA = ServiceSignal(self._svc, "IPCIRQ_IRQ_IMU_DATA", decoderFn=_scannereventsservice_ipcirq_irq_imu_data_decoder)
        self._IPCIRQ_IRQ_GPS_PPS_DATA = ServiceSignal(self._svc, "IPCIRQ_IRQ_GPS_PPS_DATA", decoderFn=_scannereventsservice_ipcirq_irq_gps_pps_data_decoder)
        self._IPCIRQ_IRQ_LOCAL_TIME_UPDATE = ServiceSignal(self._svc, "IPCIRQ_IRQ_LOCAL_TIME_UPDATE")
        self._IPCIRQ_IRQ_HALT_TEGRA_REQ = ServiceSignal(self._svc, "IPCIRQ_IRQ_HALT_TEGRA_REQ")
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()


    def error(self):
        """The error signal allows services to report problems that are
           unrelated to service calls. It was intended to allow services
           to report hardware related problems. But since most services
           provide some kind of business logic, this signal is rarely used.
           
           Returns: ServiceSignal
           Payload: dict"""
        return self._error

    def IPCIRQ_IRQ_SCAN_START(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_SCAN_START

    def IPCIRQ_IRQ_SCAN_DONE(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_SCAN_DONE

    def IPCIRQ_IRQ_MEAS_READY(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_MEAS_READY

    def IPCIRQ_IRQ_MEAS_ACQ_START(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_MEAS_ACQ_START

    def IPCIRQ_IRQ_MEAS_ACQ_STOP(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_MEAS_ACQ_STOP

    def IPCIRQ_IRQ_MEAS_ACQ_DONE(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_MEAS_ACQ_DONE

    def IPCIRQ_IRQ_MEAS_ACQ_ABORT(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_MEAS_ACQ_ABORT

    def IPCIRQ_IRQ_PROPERTY_UPDATE(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_PROPERTY_UPDATE

    def IPCIRQ_IRQ_ERROR(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_ERROR

    def IPCIRQ_IRQ_ERROR_PUSH(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_ERROR_PUSH

    def IPCIRQ_IRQ_ERROR_ACK(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_ERROR_ACK

    def IPCIRQ_IRQ_SEQUENCE(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_SEQUENCE

    def IPCIRQ_IRQ_SEQUENCE_DONE_OK(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_SEQUENCE_DONE_OK

    def IPCIRQ_IRQ_SEQUENCE_FAILED(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_SEQUENCE_FAILED

    def IPCIRQ_IRQ_IMG_ACQ_START(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_IMG_ACQ_START

    def IPCIRQ_IRQ_IMG_ACQ_STOP(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_IMG_ACQ_STOP

    def IPCIRQ_IRQ_IMG_ACQ_DONE(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_IMG_ACQ_DONE

    def IPCIRQ_IRQ_IMG_UPDATE(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_IMG_UPDATE

    def IPCIRQ_IRQ_IMG_EXPOSE(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_IMG_EXPOSE

    def IPCIRQ_IRQ_IMG_ACQ_CMD(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_IMG_ACQ_CMD

    def IPCIRQ_IRQ_SHUTDOWN_REQ(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_SHUTDOWN_REQ

    def IPCIRQ_IRQ_FRAME_UPSCAN(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_FRAME_UPSCAN

    def IPCIRQ_IRQ_FRAME_DOWNSCAN(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_FRAME_DOWNSCAN

    def IPCIRQ_IRQ_CAMERA_MOUNT_DETECTION(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_CAMERA_MOUNT_DETECTION

    def IPCIRQ_IRQ_IMU_DATA(self):
        """Returns: ServiceSignal
           Payload: imu_data"""
        return self._IPCIRQ_IRQ_IMU_DATA

    def IPCIRQ_IRQ_GPS_PPS_DATA(self):
        """Returns: ServiceSignal
           Payload: gps_pps_data"""
        return self._IPCIRQ_IRQ_GPS_PPS_DATA

    def IPCIRQ_IRQ_LOCAL_TIME_UPDATE(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_LOCAL_TIME_UPDATE

    def IPCIRQ_IRQ_HALT_TEGRA_REQ(self):
        """Returns: ServiceSignal"""
        return self._IPCIRQ_IRQ_HALT_TEGRA_REQ
