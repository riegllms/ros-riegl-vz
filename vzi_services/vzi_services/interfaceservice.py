# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.15"

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


class NetworkInterfaceState(dict):
    def __init__(self, *args, **kwargs):
        self['is_intf_up'] = None
        self['is_link_up'] = None
        self['speed'] = None
        self['wifi_mode'] = None
        self['wifi_essid'] = None
        self['wifi_quality'] = None
        self['wifi_rssi'] = None
        self['wifi_bit_rate'] = None
        self['wifi_country_code'] = None
        self['wifi_channel'] = None
        self['wifi_freq'] = None
        super().__init__(*args, **kwargs)
    @property
    def is_intf_up(self):
        return self['is_intf_up']
    @is_intf_up.setter
    def is_intf_up(self, value):
        self['is_intf_up'] = value
    @property
    def is_link_up(self):
        return self['is_link_up']
    @is_link_up.setter
    def is_link_up(self, value):
        self['is_link_up'] = value
    @property
    def speed(self):
        return self['speed']
    @speed.setter
    def speed(self, value):
        self['speed'] = value
    @property
    def wifi_mode(self):
        return self['wifi_mode']
    @wifi_mode.setter
    def wifi_mode(self, value):
        self['wifi_mode'] = value
    @property
    def wifi_essid(self):
        return self['wifi_essid']
    @wifi_essid.setter
    def wifi_essid(self, value):
        self['wifi_essid'] = value
    @property
    def wifi_quality(self):
        return self['wifi_quality']
    @wifi_quality.setter
    def wifi_quality(self, value):
        self['wifi_quality'] = value
    @property
    def wifi_rssi(self):
        return self['wifi_rssi']
    @wifi_rssi.setter
    def wifi_rssi(self, value):
        self['wifi_rssi'] = value
    @property
    def wifi_bit_rate(self):
        return self['wifi_bit_rate']
    @wifi_bit_rate.setter
    def wifi_bit_rate(self, value):
        self['wifi_bit_rate'] = value
    @property
    def wifi_country_code(self):
        return self['wifi_country_code']
    @wifi_country_code.setter
    def wifi_country_code(self, value):
        self['wifi_country_code'] = value
    @property
    def wifi_channel(self):
        return self['wifi_channel']
    @wifi_channel.setter
    def wifi_channel(self, value):
        self['wifi_channel'] = value
    @property
    def wifi_freq(self):
        return self['wifi_freq']
    @wifi_freq.setter
    def wifi_freq(self, value):
        self['wifi_freq'] = value

class IP4Network(dict):
    def __init__(self, *args, **kwargs):
        self['ip'] = None
        self['subnet'] = None
        self['gateway'] = None
        super().__init__(*args, **kwargs)
    @property
    def ip(self):
        return self['ip']
    @ip.setter
    def ip(self, value):
        self['ip'] = value
    @property
    def subnet(self):
        return self['subnet']
    @subnet.setter
    def subnet(self, value):
        self['subnet'] = value
    @property
    def gateway(self):
        return self['gateway']
    @gateway.setter
    def gateway(self, value):
        self['gateway'] = value

class IP4Route(dict):
    def __init__(self, *args, **kwargs):
        self['source'] = None
        self['destination'] = None
        self['gateway'] = None
        super().__init__(*args, **kwargs)
    @property
    def source(self):
        return self['source']
    @source.setter
    def source(self, value):
        self['source'] = value
    @property
    def destination(self):
        return self['destination']
    @destination.setter
    def destination(self, value):
        self['destination'] = value
    @property
    def gateway(self):
        return self['gateway']
    @gateway.setter
    def gateway(self, value):
        self['gateway'] = value

class IP6Network(dict):
    def __init__(self, *args, **kwargs):
        self['ip'] = None
        self['subnet'] = None
        self['gateway'] = None
        super().__init__(*args, **kwargs)
    @property
    def ip(self):
        return self['ip']
    @ip.setter
    def ip(self, value):
        self['ip'] = value
    @property
    def subnet(self):
        return self['subnet']
    @subnet.setter
    def subnet(self, value):
        self['subnet'] = value
    @property
    def gateway(self):
        return self['gateway']
    @gateway.setter
    def gateway(self, value):
        self['gateway'] = value

class IP6Route(dict):
    def __init__(self, *args, **kwargs):
        self['source'] = None
        self['destination'] = None
        self['gateway'] = None
        super().__init__(*args, **kwargs)
    @property
    def source(self):
        return self['source']
    @source.setter
    def source(self, value):
        self['source'] = value
    @property
    def destination(self):
        return self['destination']
    @destination.setter
    def destination(self, value):
        self['destination'] = value
    @property
    def gateway(self):
        return self['gateway']
    @gateway.setter
    def gateway(self, value):
        self['gateway'] = value

class WIFIconfig(dict):
    def __init__(self, *args, **kwargs):
        self['essid'] = None
        self['key'] = None
        self['channel'] = None
        super().__init__(*args, **kwargs)
    @property
    def essid(self):
        return self['essid']
    @essid.setter
    def essid(self, value):
        self['essid'] = value
    @property
    def key(self):
        return self['key']
    @key.setter
    def key(self, value):
        self['key'] = value
    @property
    def channel(self):
        return self['channel']
    @channel.setter
    def channel(self, value):
        self['channel'] = value

class IPconfig(dict):
    def __init__(self, *args, **kwargs):
        self['eMode'] = None
        self['ip4networks'] = []
        self['ip4routes'] = []
        self['ip4defaultgateway'] = None
        self['ip6networks'] = []
        self['ip6routes'] = []
        self['ip4nameservers'] = []
        self['ip6nameservers'] = []
        super().__init__(*args, **kwargs)
        self['ip4networks'] = [IP4Network(k) if type(k) == dict else k for k in self.get('ip4networks', [])]
        self['ip4routes'] = [IP4Route(k) if type(k) == dict else k for k in self.get('ip4routes', [])]
        self['ip6networks'] = [IP6Network(k) if type(k) == dict else k for k in self.get('ip6networks', [])]
        self['ip6routes'] = [IP6Route(k) if type(k) == dict else k for k in self.get('ip6routes', [])]
    @property
    def eMode(self):
        return self['eMode']
    @eMode.setter
    def eMode(self, value):
        self['eMode'] = value
    @property
    def ip4networks(self):
        return self['ip4networks']
    @ip4networks.setter
    def ip4networks(self, value):
        self['ip4networks'] = value
    @property
    def ip4routes(self):
        return self['ip4routes']
    @ip4routes.setter
    def ip4routes(self, value):
        self['ip4routes'] = value
    @property
    def ip4defaultgateway(self):
        return self['ip4defaultgateway']
    @ip4defaultgateway.setter
    def ip4defaultgateway(self, value):
        self['ip4defaultgateway'] = value
    @property
    def ip6networks(self):
        return self['ip6networks']
    @ip6networks.setter
    def ip6networks(self, value):
        self['ip6networks'] = value
    @property
    def ip6routes(self):
        return self['ip6routes']
    @ip6routes.setter
    def ip6routes(self, value):
        self['ip6routes'] = value
    @property
    def ip4nameservers(self):
        return self['ip4nameservers']
    @ip4nameservers.setter
    def ip4nameservers(self, value):
        self['ip4nameservers'] = value
    @property
    def ip6nameservers(self):
        return self['ip6nameservers']
    @ip6nameservers.setter
    def ip6nameservers(self, value):
        self['ip6nameservers'] = value

class NetworkInterface(dict):
    def __init__(self, *args, **kwargs):
        self['name'] = None
        self['eType'] = None
        self['mac'] = None
        self['ip_config'] = None
        self['wifi_config'] = None
        self['state'] = None
        super().__init__(*args, **kwargs)
        if type(self.get('ip_config')) == dict: self['ip_config'] = IPconfig(self.get('ip_config'))
        if type(self.get('wifi_config')) == dict: self['wifi_config'] = WIFIconfig(self.get('wifi_config'))
        if type(self.get('state')) == dict: self['state'] = NetworkInterfaceState(self.get('state'))
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def eType(self):
        return self['eType']
    @eType.setter
    def eType(self, value):
        self['eType'] = value
    @property
    def mac(self):
        return self['mac']
    @mac.setter
    def mac(self, value):
        self['mac'] = value
    @property
    def ip_config(self):
        return self['ip_config']
    @ip_config.setter
    def ip_config(self, value):
        self['ip_config'] = value
    @property
    def wifi_config(self):
        return self['wifi_config']
    @wifi_config.setter
    def wifi_config(self, value):
        self['wifi_config'] = value
    @property
    def state(self):
        return self['state']
    @state.setter
    def state(self, value):
        self['state'] = value

class WIFInetwork(dict):
    def __init__(self, *args, **kwargs):
        self['essid'] = None
        self['quality'] = None
        self['encryptMode'] = None
        super().__init__(*args, **kwargs)
    @property
    def essid(self):
        return self['essid']
    @essid.setter
    def essid(self, value):
        self['essid'] = value
    @property
    def quality(self):
        return self['quality']
    @quality.setter
    def quality(self, value):
        self['quality'] = value
    @property
    def encryptMode(self):
        return self['encryptMode']
    @encryptMode.setter
    def encryptMode(self, value):
        self['encryptMode'] = value

class WIFIchannel(dict):
    def __init__(self, *args, **kwargs):
        self['number'] = None
        self['desc'] = None
        super().__init__(*args, **kwargs)
    @property
    def number(self):
        return self['number']
    @number.setter
    def number(self, value):
        self['number'] = value
    @property
    def desc(self):
        return self['desc']
    @desc.setter
    def desc(self, value):
        self['desc'] = value

class InetConnStatus(dict):
    def __init__(self, *args, **kwargs):
        self['intf_name'] = None
        self['gateway'] = None
        self['online'] = None
        super().__init__(*args, **kwargs)
    @property
    def intf_name(self):
        return self['intf_name']
    @intf_name.setter
    def intf_name(self, value):
        self['intf_name'] = value
    @property
    def gateway(self):
        return self['gateway']
    @gateway.setter
    def gateway(self, value):
        self['gateway'] = value
    @property
    def online(self):
        return self['online']
    @online.setter
    def online(self, value):
        self['online'] = value

class StorageSpace(dict):
    def __init__(self, *args, **kwargs):
        self['total_space'] = None
        self['used_space'] = None
        super().__init__(*args, **kwargs)
    @property
    def total_space(self):
        return self['total_space']
    @total_space.setter
    def total_space(self, value):
        self['total_space'] = value
    @property
    def used_space(self):
        return self['used_space']
    @used_space.setter
    def used_space(self, value):
        self['used_space'] = value

class MountPoint(dict):
    def __init__(self, *args, **kwargs):
        self['device_file'] = None
        self['path'] = None
        self['fs_type'] = None
        self['storage_space'] = None
        self['mount_spec'] = None
        super().__init__(*args, **kwargs)
        if type(self.get('storage_space')) == dict: self['storage_space'] = StorageSpace(self.get('storage_space'))
    @property
    def device_file(self):
        return self['device_file']
    @device_file.setter
    def device_file(self, value):
        self['device_file'] = value
    @property
    def path(self):
        return self['path']
    @path.setter
    def path(self, value):
        self['path'] = value
    @property
    def fs_type(self):
        return self['fs_type']
    @fs_type.setter
    def fs_type(self, value):
        self['fs_type'] = value
    @property
    def storage_space(self):
        return self['storage_space']
    @storage_space.setter
    def storage_space(self, value):
        self['storage_space'] = value
    @property
    def mount_spec(self):
        return self['mount_spec']
    @mount_spec.setter
    def mount_spec(self, value):
        self['mount_spec'] = value

class StorageInterface(dict):
    def __init__(self, *args, **kwargs):
        self['name'] = None
        self['eType'] = None
        self['vendor'] = None
        self['model'] = None
        self['serialno'] = None
        self['mounts'] = []
        self['is_removable'] = None
        super().__init__(*args, **kwargs)
        self['mounts'] = [MountPoint(k) if type(k) == dict else k for k in self.get('mounts', [])]
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def eType(self):
        return self['eType']
    @eType.setter
    def eType(self, value):
        self['eType'] = value
    @property
    def vendor(self):
        return self['vendor']
    @vendor.setter
    def vendor(self, value):
        self['vendor'] = value
    @property
    def model(self):
        return self['model']
    @model.setter
    def model(self, value):
        self['model'] = value
    @property
    def serialno(self):
        return self['serialno']
    @serialno.setter
    def serialno(self, value):
        self['serialno'] = value
    @property
    def mounts(self):
        return self['mounts']
    @mounts.setter
    def mounts(self, value):
        self['mounts'] = value
    @property
    def is_removable(self):
        return self['is_removable']
    @is_removable.setter
    def is_removable(self, value):
        self['is_removable'] = value

class NetworkStorageConfig(dict):
    def __init__(self, *args, **kwargs):
        self['protocol'] = None
        self['address'] = None
        self['root_dir'] = None
        self['user'] = None
        self['password'] = None
        self['domain'] = None
        self['options'] = None
        super().__init__(*args, **kwargs)
    @property
    def protocol(self):
        return self['protocol']
    @protocol.setter
    def protocol(self, value):
        self['protocol'] = value
    @property
    def address(self):
        return self['address']
    @address.setter
    def address(self, value):
        self['address'] = value
    @property
    def root_dir(self):
        return self['root_dir']
    @root_dir.setter
    def root_dir(self, value):
        self['root_dir'] = value
    @property
    def user(self):
        return self['user']
    @user.setter
    def user(self, value):
        self['user'] = value
    @property
    def password(self):
        return self['password']
    @password.setter
    def password(self, value):
        self['password'] = value
    @property
    def domain(self):
        return self['domain']
    @domain.setter
    def domain(self, value):
        self['domain'] = value
    @property
    def options(self):
        return self['options']
    @options.setter
    def options(self, value):
        self['options'] = value

class MobileDeviceInfo(dict):
    def __init__(self, *args, **kwargs):
        self['manufacturer'] = None
        self['model'] = None
        self['region'] = None
        self['firmware_version'] = None
        self['IMEI'] = None
        self['IMSI'] = None
        self['ICCID'] = None
        super().__init__(*args, **kwargs)
    @property
    def manufacturer(self):
        return self['manufacturer']
    @manufacturer.setter
    def manufacturer(self, value):
        self['manufacturer'] = value
    @property
    def model(self):
        return self['model']
    @model.setter
    def model(self, value):
        self['model'] = value
    @property
    def region(self):
        return self['region']
    @region.setter
    def region(self, value):
        self['region'] = value
    @property
    def firmware_version(self):
        return self['firmware_version']
    @firmware_version.setter
    def firmware_version(self, value):
        self['firmware_version'] = value
    @property
    def IMEI(self):
        return self['IMEI']
    @IMEI.setter
    def IMEI(self, value):
        self['IMEI'] = value
    @property
    def IMSI(self):
        return self['IMSI']
    @IMSI.setter
    def IMSI(self, value):
        self['IMSI'] = value
    @property
    def ICCID(self):
        return self['ICCID']
    @ICCID.setter
    def ICCID(self, value):
        self['ICCID'] = value

class MobileNetworkOperator(dict):
    def __init__(self, *args, **kwargs):
        self['oper'] = None
        self['numeric'] = None
        self['act_gen'] = None
        self['is_current'] = None
        super().__init__(*args, **kwargs)
    @property
    def oper(self):
        return self['oper']
    @oper.setter
    def oper(self, value):
        self['oper'] = value
    @property
    def numeric(self):
        return self['numeric']
    @numeric.setter
    def numeric(self, value):
        self['numeric'] = value
    @property
    def act_gen(self):
        return self['act_gen']
    @act_gen.setter
    def act_gen(self, value):
        self['act_gen'] = value
    @property
    def is_current(self):
        return self['is_current']
    @is_current.setter
    def is_current(self, value):
        self['is_current'] = value

class MobileNetworkStatus(dict):
    def __init__(self, *args, **kwargs):
        self['reg_status'] = None
        self['mode'] = None
        self['oper'] = None
        self['numeric_oper'] = None
        self['rssi'] = None
        self['ipv4_address'] = None
        self['subnet_mask'] = None
        super().__init__(*args, **kwargs)
    @property
    def reg_status(self):
        return self['reg_status']
    @reg_status.setter
    def reg_status(self, value):
        self['reg_status'] = value
    @property
    def mode(self):
        return self['mode']
    @mode.setter
    def mode(self, value):
        self['mode'] = value
    @property
    def oper(self):
        return self['oper']
    @oper.setter
    def oper(self, value):
        self['oper'] = value
    @property
    def numeric_oper(self):
        return self['numeric_oper']
    @numeric_oper.setter
    def numeric_oper(self, value):
        self['numeric_oper'] = value
    @property
    def rssi(self):
        return self['rssi']
    @rssi.setter
    def rssi(self, value):
        self['rssi'] = value
    @property
    def ipv4_address(self):
        return self['ipv4_address']
    @ipv4_address.setter
    def ipv4_address(self, value):
        self['ipv4_address'] = value
    @property
    def subnet_mask(self):
        return self['subnet_mask']
    @subnet_mask.setter
    def subnet_mask(self, value):
        self['subnet_mask'] = value

class BluetoothInterface(dict):
    def __init__(self, *args, **kwargs):
        self['name'] = None
        self['mac'] = None
        self['enabled'] = None
        super().__init__(*args, **kwargs)
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def mac(self):
        return self['mac']
    @mac.setter
    def mac(self, value):
        self['mac'] = value
    @property
    def enabled(self):
        return self['enabled']
    @enabled.setter
    def enabled(self, value):
        self['enabled'] = value

class BluetoothDevice(dict):
    def __init__(self, *args, **kwargs):
        self['mac'] = None
        self['name'] = None
        self['available'] = None
        self['connected'] = None
        super().__init__(*args, **kwargs)
    @property
    def mac(self):
        return self['mac']
    @mac.setter
    def mac(self, value):
        self['mac'] = value
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def available(self):
        return self['available']
    @available.setter
    def available(self, value):
        self['available'] = value
    @property
    def connected(self):
        return self['connected']
    @connected.setter
    def connected(self, value):
        self['connected'] = value

class BluetoothSerialPortService(dict):
    def __init__(self, *args, **kwargs):
        self['mac'] = None
        self['name'] = None
        self['channel'] = None
        self['serial_dev_id'] = None
        self['serial_dev_file'] = None
        super().__init__(*args, **kwargs)
    @property
    def mac(self):
        return self['mac']
    @mac.setter
    def mac(self, value):
        self['mac'] = value
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def channel(self):
        return self['channel']
    @channel.setter
    def channel(self, value):
        self['channel'] = value
    @property
    def serial_dev_id(self):
        return self['serial_dev_id']
    @serial_dev_id.setter
    def serial_dev_id(self, value):
        self['serial_dev_id'] = value
    @property
    def serial_dev_file(self):
        return self['serial_dev_file']
    @serial_dev_file.setter
    def serial_dev_file(self, value):
        self['serial_dev_file'] = value

def _interfaceservice_error_decoder(payload):
    return json.loads(payload.decode())
def _interfaceservice_internetconnectionstatusupdate_decoder(payload):
    return InetConnStatus(json.loads(payload.decode()))
def _interfaceservice_storageinterfaceadded_decoder(payload):
    return StorageInterface(json.loads(payload.decode()))
def _interfaceservice_storageinterfaceremoved_decoder(payload):
    return payload.decode()
def _interfaceservice_storageinterfacewillbeejected_decoder(payload):
    return payload.decode()
def _interfaceservice_storageinterfaceejected_decoder(payload):
    return payload.decode()
def _interfaceservice_mobilenetworkoperatorsearchfinished_decoder(payload):
    return struct.unpack("!I", payload)[0]
def _interfaceservice_mobilenetworkregistrationfinished_decoder(payload):
    return MobileNetworkStatus(json.loads(payload.decode()))
def _interfaceservice_bluetoothinterfacechanged_decoder(payload):
    return BluetoothInterface(json.loads(payload.decode()))
def _interfaceservice_bluetoothdevicescanfinished_decoder(payload):
    return struct.unpack("!I", payload)[0]
def _interfaceservice_bluetoothconnectionchanged_decoder(payload):
    return BluetoothDevice(json.loads(payload.decode()))

class InterfaceService(object):
    """Service for managing I/O interfaces."""

    class Error(enum.IntEnum):
        ERROR_NONE = 0
        ERROR = 1
        ERROR_UNIMPLEMENTED = 2
        ERROR_NOT_FOUND = 3
        ERROR_SYSTEM_CMD_EXEC_FAILED = 4
        ERROR_NOT_AVAILABLE = 5
        ERROR_NO_NETWORKS = 100
        ERROR_NET_CONFIG_ERROR = 101
        ERROR_NO_NETIF_WIFI = 102
        ERROR_MOBILE_SIM_DETECT_IP = 300
        ERROR_MOBILE_CMD_EXEC_FAILED = 301
        ERROR_MOBILE_PIN_IS_INVALID = 302
        ERROR_MOBILE_NETWORK_REG_FAILED = 303
        ERROR_MOBILE_NETWORK_REG_IP = 304
        ERROR_MOBILE_OPERATOR_SEARCH_FAILED = 305
        ERROR_MOBILE_OPERATOR_SEARCH_IP = 306
        ERROR_BT_ALREADY_CONNECTED = 400
        ERROR_BT_CONFIG_ERROR = 401
        ERROR_BT_SCAN_IP = 402
        ERROR_BT_SCAN_FAILED = 403
        ERROR_BT_DEVICE_NOT_AVAIL = 404
        ERROR_BT_CONNECT_FAILED = 405

    class SystemError(enum.IntEnum):
        MOBILE_PIN_IS_NOT_INTENDED_FOR_SIM = 0
        MOBILE_PIN_IS_INVALID = 1
        MOBILE_MODULE_CONFIG_ERROR = 2
        MOBILE_OPERATOR_SEARCH_FAILED = 3
        MOBILE_NETWORK_REG_FAILED = 4
        BLUETOOTH_DEVICE_SCAN_FAILED = 5

    class NetworkInterfaceType(enum.IntEnum):
        NETIF_UNKNOWN = 0
        NETIF_LOOPBACK = 1
        NETIF_ETHERNET = 2
        NETIF_WIFI = 4
        NETIF_DATACARD = 8
        NETIF_BRIDGE = 16
        NETIF_ANY = -1

    class WIFImode(enum.IntEnum):
        WIFI_MODE_UNKNOWN = 0
        WIFI_MODE_ACCESS_POINT = 1
        WIFI_MODE_STATION = 2
        WIFI_MODE_AD_HOC = 3

    class IPconfigMode(enum.IntEnum):
        IPCONFIG_NONE = 0
        IPCONFIG_DHCP = 1
        IPCONFIG_DHCP_SERVER = 2
        IPCONFIG_STATIC = 3

    class WIFIencryptMode(enum.IntEnum):
        WIFI_ENCRYPT_UNKNOWN = 0
        WIFI_ENCRYPT_NONE = 1
        WIFI_ENCRYPT_WEP = 2
        WIFI_ENCRYPT_WPA = 3
        WIFI_ENCRYPT_WPA2 = 4

    class StorageInterfaceType(enum.IntEnum):
        STORAGEIF_UNKNOWN = 0
        STORAGEIF_SSD = 1
        STORAGEIF_USB = 2
        STORAGEIF_SDCARD = 4
        STORAGEIF_MMC = 8
        STORAGEIF_NAS = 16
        STORAGEIF_ANY = -1

    class StorageFsType(enum.IntEnum):
        STORAGEIF_FS_UNKNOWN = 0
        STORAGEIF_FS_NONE = 1
        STORAGEIF_FS_EXT2 = 2
        STORAGEIF_FS_EXT3 = 3
        STORAGEIF_FS_EXT4 = 4
        STORAGEIF_FS_FAT32 = 5
        STORAGEIF_FS_EXFAT = 6
        STORAGEIF_FS_NTFS = 7

    class StorageInterfaceMountSpec(enum.IntEnum):
        STORAGEIF_MOUNT_RO = 0
        STORAGEIF_MOUNT_RW = 1

    class NetworkStorageProtocol(enum.IntEnum):
        STORAGEIF_NAS_PROTOCOL_UNKNOWN = 0
        STORAGEIF_NAS_PROTOCOL_CIFS = 1
        STORAGEIF_NAS_PROTOCOL_NFS = 2
        STORAGEIF_NAS_PROTOCOL_SFTP = 3

    class MobileNetworkRegistrationStatus(enum.IntEnum):
        MOBILEIF_STATUS_UNKNOWN = 0
        MOBILEIF_STATUS_NOT_REGISTERED = 1
        MOBILEIF_STATUS_NOT_REGISTERED_BUT_SEARCHING = 2
        MOBILEIF_STATUS_REGISTERED_HOME = 3
        MOBILEIF_STATUS_REGISTERED_ROAMING = 4
        MOBILEIF_STATUS_REGISTRATION_DENIED = 5

    class MobileNetworkMode(enum.IntEnum):
        MOBILEIF_MODE_NONE = 0
        MOBILEIF_MODE_2G_GPRS = 1
        MOBILEIF_MODE_2G_EDGE = 2
        MOBILEIF_MODE_3G_WCDMA = 3
        MOBILEIF_MODE_3G_HSDPA = 4
        MOBILEIF_MODE_3G_HSUPA = 5
        MOBILEIF_MODE_3G_HSDPA_AND_HSUPA = 6
        MOBILEIF_MODE_4G_LTE = 7

    class MobileSimSelect(enum.IntEnum):
        MOBIF_SIM_FACTORY = 0
        MOBIF_SIM_USER = 1

    ERROR_NONE = Error.ERROR_NONE
    ERROR = Error.ERROR
    ERROR_UNIMPLEMENTED = Error.ERROR_UNIMPLEMENTED
    ERROR_NOT_FOUND = Error.ERROR_NOT_FOUND
    ERROR_SYSTEM_CMD_EXEC_FAILED = Error.ERROR_SYSTEM_CMD_EXEC_FAILED
    ERROR_NOT_AVAILABLE = Error.ERROR_NOT_AVAILABLE
    ERROR_NO_NETWORKS = Error.ERROR_NO_NETWORKS
    ERROR_NET_CONFIG_ERROR = Error.ERROR_NET_CONFIG_ERROR
    ERROR_NO_NETIF_WIFI = Error.ERROR_NO_NETIF_WIFI
    ERROR_MOBILE_SIM_DETECT_IP = Error.ERROR_MOBILE_SIM_DETECT_IP
    ERROR_MOBILE_CMD_EXEC_FAILED = Error.ERROR_MOBILE_CMD_EXEC_FAILED
    ERROR_MOBILE_PIN_IS_INVALID = Error.ERROR_MOBILE_PIN_IS_INVALID
    ERROR_MOBILE_NETWORK_REG_FAILED = Error.ERROR_MOBILE_NETWORK_REG_FAILED
    ERROR_MOBILE_NETWORK_REG_IP = Error.ERROR_MOBILE_NETWORK_REG_IP
    ERROR_MOBILE_OPERATOR_SEARCH_FAILED = Error.ERROR_MOBILE_OPERATOR_SEARCH_FAILED
    ERROR_MOBILE_OPERATOR_SEARCH_IP = Error.ERROR_MOBILE_OPERATOR_SEARCH_IP
    ERROR_BT_ALREADY_CONNECTED = Error.ERROR_BT_ALREADY_CONNECTED
    ERROR_BT_CONFIG_ERROR = Error.ERROR_BT_CONFIG_ERROR
    ERROR_BT_SCAN_IP = Error.ERROR_BT_SCAN_IP
    ERROR_BT_SCAN_FAILED = Error.ERROR_BT_SCAN_FAILED
    ERROR_BT_DEVICE_NOT_AVAIL = Error.ERROR_BT_DEVICE_NOT_AVAIL
    ERROR_BT_CONNECT_FAILED = Error.ERROR_BT_CONNECT_FAILED
    MOBILE_PIN_IS_NOT_INTENDED_FOR_SIM = SystemError.MOBILE_PIN_IS_NOT_INTENDED_FOR_SIM
    MOBILE_PIN_IS_INVALID = SystemError.MOBILE_PIN_IS_INVALID
    MOBILE_MODULE_CONFIG_ERROR = SystemError.MOBILE_MODULE_CONFIG_ERROR
    MOBILE_OPERATOR_SEARCH_FAILED = SystemError.MOBILE_OPERATOR_SEARCH_FAILED
    MOBILE_NETWORK_REG_FAILED = SystemError.MOBILE_NETWORK_REG_FAILED
    BLUETOOTH_DEVICE_SCAN_FAILED = SystemError.BLUETOOTH_DEVICE_SCAN_FAILED
    NETIF_UNKNOWN = NetworkInterfaceType.NETIF_UNKNOWN
    NETIF_LOOPBACK = NetworkInterfaceType.NETIF_LOOPBACK
    NETIF_ETHERNET = NetworkInterfaceType.NETIF_ETHERNET
    NETIF_WIFI = NetworkInterfaceType.NETIF_WIFI
    NETIF_DATACARD = NetworkInterfaceType.NETIF_DATACARD
    NETIF_BRIDGE = NetworkInterfaceType.NETIF_BRIDGE
    NETIF_ANY = NetworkInterfaceType.NETIF_ANY
    WIFI_MODE_UNKNOWN = WIFImode.WIFI_MODE_UNKNOWN
    WIFI_MODE_ACCESS_POINT = WIFImode.WIFI_MODE_ACCESS_POINT
    WIFI_MODE_STATION = WIFImode.WIFI_MODE_STATION
    WIFI_MODE_AD_HOC = WIFImode.WIFI_MODE_AD_HOC
    IPCONFIG_NONE = IPconfigMode.IPCONFIG_NONE
    IPCONFIG_DHCP = IPconfigMode.IPCONFIG_DHCP
    IPCONFIG_DHCP_SERVER = IPconfigMode.IPCONFIG_DHCP_SERVER
    IPCONFIG_STATIC = IPconfigMode.IPCONFIG_STATIC
    WIFI_ENCRYPT_UNKNOWN = WIFIencryptMode.WIFI_ENCRYPT_UNKNOWN
    WIFI_ENCRYPT_NONE = WIFIencryptMode.WIFI_ENCRYPT_NONE
    WIFI_ENCRYPT_WEP = WIFIencryptMode.WIFI_ENCRYPT_WEP
    WIFI_ENCRYPT_WPA = WIFIencryptMode.WIFI_ENCRYPT_WPA
    WIFI_ENCRYPT_WPA2 = WIFIencryptMode.WIFI_ENCRYPT_WPA2
    STORAGEIF_UNKNOWN = StorageInterfaceType.STORAGEIF_UNKNOWN
    STORAGEIF_SSD = StorageInterfaceType.STORAGEIF_SSD
    STORAGEIF_USB = StorageInterfaceType.STORAGEIF_USB
    STORAGEIF_SDCARD = StorageInterfaceType.STORAGEIF_SDCARD
    STORAGEIF_MMC = StorageInterfaceType.STORAGEIF_MMC
    STORAGEIF_NAS = StorageInterfaceType.STORAGEIF_NAS
    STORAGEIF_ANY = StorageInterfaceType.STORAGEIF_ANY
    STORAGEIF_FS_UNKNOWN = StorageFsType.STORAGEIF_FS_UNKNOWN
    STORAGEIF_FS_NONE = StorageFsType.STORAGEIF_FS_NONE
    STORAGEIF_FS_EXT2 = StorageFsType.STORAGEIF_FS_EXT2
    STORAGEIF_FS_EXT3 = StorageFsType.STORAGEIF_FS_EXT3
    STORAGEIF_FS_EXT4 = StorageFsType.STORAGEIF_FS_EXT4
    STORAGEIF_FS_FAT32 = StorageFsType.STORAGEIF_FS_FAT32
    STORAGEIF_FS_EXFAT = StorageFsType.STORAGEIF_FS_EXFAT
    STORAGEIF_FS_NTFS = StorageFsType.STORAGEIF_FS_NTFS
    STORAGEIF_MOUNT_RO = StorageInterfaceMountSpec.STORAGEIF_MOUNT_RO
    STORAGEIF_MOUNT_RW = StorageInterfaceMountSpec.STORAGEIF_MOUNT_RW
    STORAGEIF_NAS_PROTOCOL_UNKNOWN = NetworkStorageProtocol.STORAGEIF_NAS_PROTOCOL_UNKNOWN
    STORAGEIF_NAS_PROTOCOL_CIFS = NetworkStorageProtocol.STORAGEIF_NAS_PROTOCOL_CIFS
    STORAGEIF_NAS_PROTOCOL_NFS = NetworkStorageProtocol.STORAGEIF_NAS_PROTOCOL_NFS
    STORAGEIF_NAS_PROTOCOL_SFTP = NetworkStorageProtocol.STORAGEIF_NAS_PROTOCOL_SFTP
    MOBILEIF_STATUS_UNKNOWN = MobileNetworkRegistrationStatus.MOBILEIF_STATUS_UNKNOWN
    MOBILEIF_STATUS_NOT_REGISTERED = MobileNetworkRegistrationStatus.MOBILEIF_STATUS_NOT_REGISTERED
    MOBILEIF_STATUS_NOT_REGISTERED_BUT_SEARCHING = MobileNetworkRegistrationStatus.MOBILEIF_STATUS_NOT_REGISTERED_BUT_SEARCHING
    MOBILEIF_STATUS_REGISTERED_HOME = MobileNetworkRegistrationStatus.MOBILEIF_STATUS_REGISTERED_HOME
    MOBILEIF_STATUS_REGISTERED_ROAMING = MobileNetworkRegistrationStatus.MOBILEIF_STATUS_REGISTERED_ROAMING
    MOBILEIF_STATUS_REGISTRATION_DENIED = MobileNetworkRegistrationStatus.MOBILEIF_STATUS_REGISTRATION_DENIED
    MOBILEIF_MODE_NONE = MobileNetworkMode.MOBILEIF_MODE_NONE
    MOBILEIF_MODE_2G_GPRS = MobileNetworkMode.MOBILEIF_MODE_2G_GPRS
    MOBILEIF_MODE_2G_EDGE = MobileNetworkMode.MOBILEIF_MODE_2G_EDGE
    MOBILEIF_MODE_3G_WCDMA = MobileNetworkMode.MOBILEIF_MODE_3G_WCDMA
    MOBILEIF_MODE_3G_HSDPA = MobileNetworkMode.MOBILEIF_MODE_3G_HSDPA
    MOBILEIF_MODE_3G_HSUPA = MobileNetworkMode.MOBILEIF_MODE_3G_HSUPA
    MOBILEIF_MODE_3G_HSDPA_AND_HSUPA = MobileNetworkMode.MOBILEIF_MODE_3G_HSDPA_AND_HSUPA
    MOBILEIF_MODE_4G_LTE = MobileNetworkMode.MOBILEIF_MODE_4G_LTE
    MOBIF_SIM_FACTORY = MobileSimSelect.MOBIF_SIM_FACTORY
    MOBIF_SIM_USER = MobileSimSelect.MOBIF_SIM_USER

    def __init__(self, address):
        self._svc = riconnect.Service("InterfaceService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_interfaceservice_error_decoder)
        self._internetConnectionStatusUpdate = ServiceSignal(self._svc, "internetConnectionStatusUpdate", decoderFn=_interfaceservice_internetconnectionstatusupdate_decoder)
        self._storageInterfaceAdded = ServiceSignal(self._svc, "storageInterfaceAdded", decoderFn=_interfaceservice_storageinterfaceadded_decoder)
        self._storageInterfaceRemoved = ServiceSignal(self._svc, "storageInterfaceRemoved", decoderFn=_interfaceservice_storageinterfaceremoved_decoder)
        self._storageInterfaceWillBeEjected = ServiceSignal(self._svc, "storageInterfaceWillBeEjected", decoderFn=_interfaceservice_storageinterfacewillbeejected_decoder)
        self._storageInterfaceEjected = ServiceSignal(self._svc, "storageInterfaceEjected", decoderFn=_interfaceservice_storageinterfaceejected_decoder)
        self._mobileNetworkOperatorSearchFinished = ServiceSignal(self._svc, "mobileNetworkOperatorSearchFinished", decoderFn=_interfaceservice_mobilenetworkoperatorsearchfinished_decoder)
        self._mobileNetworkRegistrationFinished = ServiceSignal(self._svc, "mobileNetworkRegistrationFinished", decoderFn=_interfaceservice_mobilenetworkregistrationfinished_decoder)
        self._bluetoothInterfaceChanged = ServiceSignal(self._svc, "bluetoothInterfaceChanged", decoderFn=_interfaceservice_bluetoothinterfacechanged_decoder)
        self._bluetoothDeviceScanFinished = ServiceSignal(self._svc, "bluetoothDeviceScanFinished", decoderFn=_interfaceservice_bluetoothdevicescanfinished_decoder)
        self._bluetoothConnectionChanged = ServiceSignal(self._svc, "bluetoothConnectionChanged", decoderFn=_interfaceservice_bluetoothconnectionchanged_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def getNetworkInterface(self, name):
        """Get the specified network Interface.
           
           Arguments:
             name (str): 
           
           Returns: NetworkInterface
             A NetworkInterface on success. On error a NetworkInterface with an empty name is returned."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkInterface_71a78690", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(NetworkInterface(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getNetworkInterfaces(self, eType):
        """Gets a list of network Interfaces.
           
           Arguments:
             eType (NetworkInterfaceType): Network interface type mask, e.g. NETIF_ETHERNET|NETIF_WIFI or NETIF_ANY etc.
           
           Returns: list(NetworkInterface)
                 A vector of netif objects to describe each network interface requested."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(eType)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkInterfaces_74629675", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([NetworkInterface(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getNetworkInterfaceState(self, name):
        """Gets the current network interface state.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces()
           
           Returns: NetworkInterfaceState
                    An object to describe the lan link status, lan link speed, wlan link quality, ..."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkInterfaceState_ef139fb8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(NetworkInterfaceState(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def NetworkHostname(self):
        """Get the machines network hostname.
           
           Returns: str
             The network hostname string."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("NetworkHostname_97d5a180", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def disableNetworkInterface(self, name, disable):
        """Deactivate/Activate network interface.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces()
             disable (bool): Set inteface up/down.
           
           Returns: Error
             ERROR_NONE  On success."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = name
        inputs[1].i32 = 1 if disable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("disableNetworkInterface_59f66dc0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setNetworkInterfaceConfig(self, *args):
        """
           setNetworkInterfaceConfig(name, ipconfig)
               Allows configuration of a network interface.
               
               Arguments:
                 name (str): Interface name, as returned in a netif object from getNetworkInterfaces();
                 ipconfig (IPconfig): Configuration data for the interface's IP settings.
               
               Returns: Error
                    ERROR_NONE     On success.

           setNetworkInterfaceConfig(name, ipconfig, wifiConfig)
               Allows configuration of a WIFI network interface.
               
               Arguments:
                 name (str): Interface name, as returned in a netif object from getNetworkInterfaces();
                 ipconfig (IPconfig): Configuration data for the interface's IP settings.
                 wifiConfig (WIFIconfig): 
               
               Returns: Error
                    ERROR_NONE     On success.

        """
        if len(args) == 2:
            return self._setNetworkInterfaceConfig_072f61ad(*args)
        elif len(args) == 3:
            return self._setNetworkInterfaceConfig_ffcec136(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _setNetworkInterfaceConfig_072f61ad(self, name, ipconfig):
        """Allows configuration of a network interface.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces();
             ipconfig (IPconfig): Configuration data for the interface's IP settings.
           
           Returns: Error
                ERROR_NONE     On success."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = name
        inputs[1].s = json.dumps(ipconfig)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setNetworkInterfaceConfig_072f61ad", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _setNetworkInterfaceConfig_ffcec136(self, name, ipconfig, wifiConfig):
        """Allows configuration of a WIFI network interface.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces();
             ipconfig (IPconfig): Configuration data for the interface's IP settings.
             wifiConfig (WIFIconfig): 
           
           Returns: Error
                ERROR_NONE     On success."""
        inputs = [riconnect.Value() for i in range(0, 3)]
        inputs[0].s = name
        inputs[1].s = json.dumps(ipconfig)
        inputs[2].s = json.dumps(wifiConfig)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setNetworkInterfaceConfig_ffcec136", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getNetworkInterfaceConfig(self, name, eMode):
        """Get the specified network interface configuration.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces();
             eMode (IPconfigMode): IP configuration mode.
           
           Returns: NetworkInterface
             A NetworkInterface on success. On error a NetworkInterface with an empty name is returned."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = name
        inputs[1].i32 = int(eMode)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkInterfaceConfig_67a19aec", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(NetworkInterface(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getNetworkInterfaceConfigs(self, eType, eMode):
        """Returns configuration of network interfaces.
           
           Arguments:
             eType (NetworkInterfaceType): Network interface type mask, e.g. NETIF_ETHERNET|NETIF_WIFI or NETIF_ANY etc.
             eMode (IPconfigMode): IP configuration mode.
           
           Returns: list(NetworkInterface)
                     A map of objects to describe each configured network interface."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = int(eType)
        inputs[1].i32 = int(eMode)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkInterfaceConfigs_dc48b07e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([NetworkInterface(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def scanWifiNetworks(self, name):
        """Gets a list of available WIFI Access Points.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces();
           
           Returns: list(WIFInetwork)
                    A vector of available WIFI networks."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanWifiNetworks_199de44b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=60.0)
        outputs = []

        outputs.append([WIFInetwork(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def listWifiNetworkChannels(self, name):
        """Gets a list of available WIFI channels.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces()
           
           Returns: list(WIFIchannel)
                    A vector of available WIFI channels."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("listWifiNetworkChannels_a54a8a10", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([WIFIchannel(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getNetworkIpForwarding(self):
        """Get the IP forward system configuration.
           
           Returns: bool
                    IP forward mode - 1 if forwarding is enabled, 0 if not."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkIpForwarding_7fe33eec", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setNetworkIpForwarding(self, enable):
        """Set the IP forward system configuratin.
           
           Arguments:
             enable (bool): Enable IP forwarding."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setNetworkIpForwarding_bed0b705", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getPreferredInternetRoute(self):
        """Get the preferred interface for route to internet.
           
           Returns: str
                    The interface name."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getPreferredInternetRoute_c591e075", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setPreferredInternetRoute(self, name):
        """Set the preferred interface for route to internet.
           
           Arguments:
             name (str): The interface name."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setPreferredInternetRoute_f8da77e6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getInternetConnectionStatus(self):
        """Get the internet connection status.
           
           Returns: InetConnStatus
                    An object describing the internet connection, containing the connection state and the interface used for routing."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getInternetConnectionStatus_aaad00f6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InetConnStatus(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setInternetProxy(self, server_name, port_number):
        """Set http/shttp/ftp/rsync proxy.
           
           Arguments:
             server_name (str): The proxy server name or ip address, with optional user and password <user>:<password>@<name>:<port>. e.g. user:user@192.168.1.1
             port_number (int): The proxy server port number."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = server_name
        inputs[1].u32 = port_number
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setInternetProxy_8069e116", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def setInternetProxyEnable(self, enable):
        """Enable internet proxy server configuration.
           
           Arguments:
             enable (bool): Enable/disable proxy settings."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setInternetProxyEnable_d7328bd2", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getInternetProxyConfig(self):
        """Get http/shttp/ftp/rsync proxy server configuration.
           
           Returns: tuple(str, int, bool)
             1: The proxy server name or ip address, with optional user and password <user>:<password>@<name>:<port>. e.g. user:user@192.168.1.1
             2: The proxy server port number.
             3: True if proxy configuration is enabled."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getInternetProxyConfig_b2e33bd5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(rvalues[0].s)
        outputs.append(rvalues[1].u32)
        outputs.append(True if rvalues[2].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setNetworkMtu(self, name, mtu):
        """Set MTU (Maximum Transmission Unit) for network interface.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces().
             mtu (int): Maximum Transmission Unit.
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = name
        inputs[1].u32 = mtu
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setNetworkMtu_20b0d0a9", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getNetworkMtu(self, name):
        """Get MTU (Maximum Transmission Unit) for network interface.
           
           Arguments:
             name (str): Interface name, as returned in a netif object from getNetworkInterfaces().
           
           Returns: int
             Maximum Transmission Unit."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkMtu_9d90e3ed", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setNetworkControlPortsEnable(self, enable):
        """Activates/Deactivates network control ports (20000/Riconnect, 20002/Rid-Interpreter).
           
           Arguments:
             enable (bool): Enable/disable network control ports.
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setNetworkControlPortsEnable_79c01398", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getNetworkControlPortsEnable(self):
        """Check if network control ports (20000/Riconnect, 20002/Rid-Interpreter) are enabled.
           
           Returns: bool
             True if etwork control ports are enabled, otherwise false."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkControlPortsEnable_cecc6ec5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getStorageInterfaces(self, eType):
        """Get a list of the available storage interfaces.
           
           Arguments:
             eType (StorageInterfaceType): Filter the list by @see storageif_type. E.g. STORAGEIF_USB | STORAGEIF_SDCARD or STORAGEIF_ANY
           
           Returns: list(StorageInterface)
             List of storageif objects."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(eType)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getStorageInterfaces_0e074df9", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([StorageInterface(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getStorageInterface(self, name):
        """Get a StorageInterface by its name.
           
           Arguments:
             name (str): Name of the storage interface to get.
           
           Returns: StorageInterface
             A StorageInterface on success, on Error the returned StorageInterface will have no name."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getStorageInterface_a8f363f3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(StorageInterface(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setNetworkStorageConfig(self, config):
        """Set network storage configuration.
           
           Arguments:
             config (NetworkStorageConfig): The network storage configuration data.
           
           Returns: NetworkStorageConfig
             The network storage configuration data."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = json.dumps(config)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setNetworkStorageConfig_0799b189", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(NetworkStorageConfig(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getNetworkStorageConfig(self):
        """Get network storage configuration.
           
           return The network storage configuration data.
           
           Returns: NetworkStorageConfig"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getNetworkStorageConfig_0a64159c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(NetworkStorageConfig(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def mountNetworkStorageInterface(self):
        """Mount network storage interface.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mountNetworkStorageInterface_6590a455", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=30.0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def unmountStorageInterface(self, mount_path):
        """Unmount storage interface partition.
           
           Arguments:
             mount_path (str): Path of the mount-point.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = mount_path
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("unmountStorageInterface_05e24e4c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def ejectStorageInterface(self, name):
        """Eject storage interface device.
           
           Arguments:
             name (str): 
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("ejectStorageInterface_4bd75402", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def mobileAPN(self):
        """Set mobile APN (Access Point Name).
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mobileAPN_889b996f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMobileAPN(self, apn):
        """Arguments:
             apn (str): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = apn
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMobileAPN_fdae079c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getMobileDeviceInfo(self):
        """Read mobile device info (manufacturer, serial number, firmware version, ...).
           
           Returns: tuple(Error, MobileDeviceInfo)
             1: ERROR_NONE On success.
             2: A reference to an object containing the resulting data."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMobileDeviceInfo_ade54881", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        outputs.append(MobileDeviceInfo(json.loads(rvalues[1].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def detectMobileSIM(self):
        """Detect if mobile SIM card has been inserted.
           
           Returns: bool
             True if detected."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("detectMobileSIM_eb1aa40c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMobilePIN(self, pin, remember):
        """Set PIN for mobile SIM.
           
           Arguments:
             pin (str): 4-digit PIN code.
             remember (bool): Remember PIN across device reboots.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = pin
        inputs[1].i32 = 1 if remember else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMobilePIN_7814a793", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMobilePUK(self, puk, newpin):
        """Set PUK for mobile SIM. If a wrong PIN is given three times, the PUK must be inserted in place  of the PIN, followed by the <newpin> which replaces the old pin in the SIM.
           
           Arguments:
             puk (str): 4-digit PUK code.
             newpin (str): New 4-digit PIN code, replaces the old PIN.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = puk
        inputs[1].s = newpin
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMobilePUK_d9b4c9a8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getMobilePINStatus(self):
        """Read mobile PIN/PUK status and attempt counters.
           
           Returns: tuple(Error, str, int, int, int, int)
             1: ERROR_NONE On success.
             2: String identifying the mobile PIN status (READY, SIM PIN, SIM PUK, SIM PIN2, SIM PUK2)
             3: Number of remaining attempts to enter PIN.
             4: Number of remaining attempts to enter PIN2.
             5: Number of remaining attempts to enter PUK.
             6: Number of remaining attempts to enter PUK2."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMobilePINStatus_2605c841", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        outputs.append(rvalues[1].s)
        outputs.append(rvalues[2].i32)
        outputs.append(rvalues[3].i32)
        outputs.append(rvalues[4].i32)
        outputs.append(rvalues[5].i32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def mobileNetworkOperatorName(self):
        """Select the mobile network operator.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mobileNetworkOperatorName_68da4acf", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def mobileNetworkOperatorNumeric(self):
        """Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mobileNetworkOperatorNumeric_a3edaea0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def mobileNetworkOperatorAutomatic(self):
        """Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mobileNetworkOperatorAutomatic_b80ad78c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMobileNetworkOperator(self, automatic, numeric_oper):
        """Arguments:
             automatic (bool): 
             numeric_oper (str): """
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = 1 if automatic else 0
        inputs[1].s = numeric_oper
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMobileNetworkOperator_ffd4583f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def mobileNetworkOperatorSearch(self, block):
        """Mobile network operator search.
           
           Arguments:
             block (bool): Block until search has finished, or send a finished signal.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if block else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mobileNetworkOperatorSearch_9ac9149e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=240.0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getMobileNetworkOperators(self):
        """Get a list of available network operators.
           
           Returns: list(MobileNetworkOperator)
             A list of network operator descriptors."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMobileNetworkOperators_1751a8f3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([MobileNetworkOperator(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def mobileNetworkRegistration(self, block):
        """Mobile network registration.
           
           Arguments:
             block (bool): Block until registration has finished, or send a finished signal.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if block else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mobileNetworkRegistration_7b04a1ac", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=150.0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getMobileNetworkStatus(self):
        """Get the mobile network registration status.
           
           Returns: tuple(Error, MobileNetworkStatus)
             1: ERROR_NONE On success.
             2: A reference to an object containing the resulting network status."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMobileNetworkStatus_d9dbd396", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        outputs.append(MobileNetworkStatus(json.loads(rvalues[1].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getMobileSignalQuality(self):
        """Get the received mobile signal quality.
           
           Returns: tuple(Error, int)
             1: ERROR_NONE On success.
             2: Received Signal Quality in percent."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMobileSignalQuality_cc980254", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        outputs.append(rvalues[1].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def mobileAirplaneMode(self):
        """Activates mobile network airplane mode.
           
           Returns: bool
             ERROR_NONE On success."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("mobileAirplaneMode_9a45317c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMobileAirplaneMode(self, enable):
        """Arguments:
             enable (bool): 
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMobileAirplaneMode_68fd06fa", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getMobileSimSelect(self):
        """Get the selected mobile SIM card.
           
           Returns: MobileSimSelect
             The automatically at startup selected SIM card, either 1 = USER (if present) or 0 = FACTORY."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMobileSimSelect_a68f7b28", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.MobileSimSelect(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getBluetoothInterface(self):
        """Get the local bluetooth interface with description.
           
           Returns: BluetoothInterface
             Object describint the local bluetooth interface."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getBluetoothInterface_316ed5fe", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(BluetoothInterface(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def bluetoothInterfaceDisable(self):
        """Get bluetooth interface state.
           
           Returns: bool
             True if disabled, false if enabled."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("bluetoothInterfaceDisable_5a8c18ec", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setBluetoothInterfaceDisable(self, disable):
        """Set bluetooth interface active or inactive.
           
           Arguments:
             disable (bool): De-/activate bluetooth interface.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if disable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setBluetoothInterfaceDisable_c96c5239", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def bluetoothDeviceScan(self, block):
        """Start bluetooth device inquisition.
           
           Arguments:
             block (bool): Blocking or non-blocking mode.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if block else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("bluetoothDeviceScan_d3f33af6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getBluetoothDevices(self):
        """Get list of discovered bluetooth devices.
           
           Returns: list(BluetoothDevice)
             Vector of bluetooth device objects."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getBluetoothDevices_a5095609", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([BluetoothDevice(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getBluetoothSerialPortServices(self, mac):
        """Get list of provided serial port services of remote bluetooth device.
           
           Arguments:
             mac (str): 
           
           Returns: list(BluetoothSerialPortService)
             Vector of bluetooth serial port service objects."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = mac
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getBluetoothSerialPortServices_dfc84d9b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([BluetoothSerialPortService(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def bluetoothConnect(self, mac, remember):
        """Connect to all supported services of remote bluetooth device.
           
           Arguments:
             mac (str): MAC address of the remote bluetooth device.
             remember (bool): Remember connection settings across device power cycles.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = mac
        inputs[1].i32 = 1 if remember else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("bluetoothConnect_f06c1cc2", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def bluetoothDisconnect(self, mac):
        """Disconnect from all services fo a remote bluetooth device .
           
           Arguments:
             mac (str): MAC address of the remote bluetooth device.
           
           Returns: Error
             ERROR_NONE On success."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = mac
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("bluetoothDisconnect_236c71a6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(InterfaceService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def savePasswords(self):
        """"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("savePasswords_22687198", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
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

    def internetConnectionStatusUpdate(self):
        """Returns: ServiceSignal
           Payload: InetConnStatus"""
        return self._internetConnectionStatusUpdate

    def storageInterfaceAdded(self):
        """Returns: ServiceSignal
           Payload: StorageInterface"""
        return self._storageInterfaceAdded

    def storageInterfaceRemoved(self):
        """Returns: ServiceSignal
           Payload: str"""
        return self._storageInterfaceRemoved

    def storageInterfaceWillBeEjected(self):
        """Returns: ServiceSignal
           Payload: str"""
        return self._storageInterfaceWillBeEjected

    def storageInterfaceEjected(self):
        """Returns: ServiceSignal
           Payload: str"""
        return self._storageInterfaceEjected

    def mobileNetworkOperatorSearchFinished(self):
        """Returns: ServiceSignal
           Payload: int"""
        return self._mobileNetworkOperatorSearchFinished

    def mobileNetworkRegistrationFinished(self):
        """Returns: ServiceSignal
           Payload: MobileNetworkStatus"""
        return self._mobileNetworkRegistrationFinished

    def bluetoothInterfaceChanged(self):
        """Returns: ServiceSignal
           Payload: BluetoothInterface"""
        return self._bluetoothInterfaceChanged

    def bluetoothDeviceScanFinished(self):
        """Returns: ServiceSignal
           Payload: int"""
        return self._bluetoothDeviceScanFinished

    def bluetoothConnectionChanged(self):
        """Returns: ServiceSignal
           Payload: BluetoothDevice"""
        return self._bluetoothConnectionChanged
