# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.2"

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


class SysError(dict):
    def __init__(self, *args, **kwargs):
        self['id'] = None
        self['timestamp'] = None
        self['code'] = None
        self['subcode'] = None
        self['severity'] = None
        self['action'] = None
        self['systime'] = None
        self['serviceName'] = None
        self['briefMessage'] = None
        self['detailMessage'] = None
        self['factoryAnnex'] = None
        super().__init__(*args, **kwargs)
    @property
    def id(self):
        return self['id']
    @id.setter
    def id(self, value):
        self['id'] = value
    @property
    def timestamp(self):
        return self['timestamp']
    @timestamp.setter
    def timestamp(self, value):
        self['timestamp'] = value
    @property
    def code(self):
        return self['code']
    @code.setter
    def code(self, value):
        self['code'] = value
    @property
    def subcode(self):
        return self['subcode']
    @subcode.setter
    def subcode(self, value):
        self['subcode'] = value
    @property
    def severity(self):
        return self['severity']
    @severity.setter
    def severity(self, value):
        self['severity'] = value
    @property
    def action(self):
        return self['action']
    @action.setter
    def action(self, value):
        self['action'] = value
    @property
    def systime(self):
        return self['systime']
    @systime.setter
    def systime(self, value):
        self['systime'] = value
    @property
    def serviceName(self):
        return self['serviceName']
    @serviceName.setter
    def serviceName(self, value):
        self['serviceName'] = value
    @property
    def briefMessage(self):
        return self['briefMessage']
    @briefMessage.setter
    def briefMessage(self, value):
        self['briefMessage'] = value
    @property
    def detailMessage(self):
        return self['detailMessage']
    @detailMessage.setter
    def detailMessage(self, value):
        self['detailMessage'] = value
    @property
    def factoryAnnex(self):
        return self['factoryAnnex']
    @factoryAnnex.setter
    def factoryAnnex(self, value):
        self['factoryAnnex'] = value

class Message(dict):
    def __init__(self, *args, **kwargs):
        self['name'] = None
        self['category'] = None
        self['text'] = None
        super().__init__(*args, **kwargs)
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def category(self):
        return self['category']
    @category.setter
    def category(self, value):
        self['category'] = value
    @property
    def text(self):
        return self['text']
    @text.setter
    def text(self, value):
        self['text'] = value

def _riconnectswitch_error_decoder(payload):
    return json.loads(payload.decode())
def _riconnectswitch_addnode_decoder(payload):
    return payload.decode()
def _riconnectswitch_removenode_decoder(payload):
    return payload.decode()
def _riconnectswitch_syserror_decoder(payload):
    return json.loads(payload.decode())

class RiconnectSwitch(object):
    """The RiconnectSwitch service."""

    class SysErrorSeverity(enum.IntEnum):
        INFO = 0
        DEFAULT = 1
        CRITICAL = 5

    INFO = SysErrorSeverity.INFO
    DEFAULT = SysErrorSeverity.DEFAULT
    CRITICAL = SysErrorSeverity.CRITICAL

    def __init__(self, address):
        self._svc = riconnect.Service("RiconnectSwitch")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_riconnectswitch_error_decoder)
        self._addNode = ServiceSignal(self._svc, "addNode", decoderFn=_riconnectswitch_addnode_decoder)
        self._removeNode = ServiceSignal(self._svc, "removeNode", decoderFn=_riconnectswitch_removenode_decoder)
        self._syserror = ServiceSignal(self._svc, "syserror", decoderFn=_riconnectswitch_syserror_decoder)
        self._sysErrorAck = ServiceSignal(self._svc, "sysErrorAck")
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def ping(self, serviceName):
        """Ping RiConnect network nodes by name.
           
           Arguments:
             serviceName (str): The name of the service network node.
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = serviceName
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("ping_ace41814", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def numSysErrors(self):
        """Return the number of errors on the error stack.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("numSysErrors_b2e5d228", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def numCriticalSysErrors(self):
        """Return the number of critical errors on the error stack.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("numCriticalSysErrors_548a6792", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def readSysErrors(self):
        """Return the system errors on the error stack.
           
           Returns: list(SysError)"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("readSysErrors_b40c2333", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([SysError(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def pushSysError(self, code, severity, serviceName, briefMessage):
        """Push a system error onto the error stack.
           
           Returns true if the error was successfully pushed on the error stack.
           
           Arguments:
             code (int): System error code.
             severity (SysErrorSeverity): System error severity.
             serviceName (str): Name of the module pushing the error.
             briefMessage (str): Error message text.
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].u32 = code
        inputs[1].i32 = int(severity)
        inputs[2].s = serviceName
        inputs[3].s = briefMessage
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("pushSysError_89658fbc", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def ackSysError(self, id, force):
        """Acknowledge and remove a single error from the error stack.
           
           Arguments:
             id (int): Unique error identifier.
             force (bool): Force acknowledgement of critical errors.
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].u32 = id
        inputs[1].i32 = 1 if force else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("ackSysError_d5295cc5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def ackSysErrors(self, force):
        """Acknowledge and remove all errors from the error stack.
           
           Arguments:
             force (bool): Force acknowledgement of critical errors.
           
           Returns: int"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if force else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("ackSysErrors_06bea832", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def createPort(self, port_desc):
        """createPort Create riconnect switch listening server port.
           
           Arguments:
             port_desc (str): Port descriptor, eg. eth0:20000, 127.0.0.1:20000.
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = port_desc
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("createPort_681d0f41", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=30.0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def destroyPort(self, port_desc):
        """destroyPort Destroy riconnect switch listening server port.
           
           Arguments:
             port_desc (str): Port descriptor, eg. eth0:20000, 127.0.0.1:20000."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = port_desc
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("destroyPort_57e70578", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getMessages(self, *args):
        """
           getMessages()
               Return all messages from the message box.
               
               Returns: list(Message)

           getMessages(category)
               Return messages from the message box.
               
               Arguments:
                 category (str): The message category.
               
               Returns: list(Message)

           getMessages(category, name)
               Return a messages from the message box.
               
               Arguments:
                 category (str): The message category.
                 name (str): The message name.
               
               Returns: list(Message)

        """
        if len(args) == 0:
            return self._getMessages_486a976e(*args)
        elif len(args) == 1:
            return self._getMessages_9ac9743f(*args)
        elif len(args) == 2:
            return self._getMessages_89fcf63d(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _getMessages_486a976e(self):
        """Return all messages from the message box.
           
           Returns: list(Message)"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMessages_486a976e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([Message(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _getMessages_9ac9743f(self, category):
        """Return messages from the message box.
           
           Arguments:
             category (str): The message category.
           
           Returns: list(Message)"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = category
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMessages_9ac9743f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([Message(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _getMessages_89fcf63d(self, category, name):
        """Return a messages from the message box.
           
           Arguments:
             category (str): The message category.
             name (str): The message name.
           
           Returns: list(Message)"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = category
        inputs[1].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getMessages_89fcf63d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([Message(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setMessage(self, message):
        """Send text messages to message box..
           
           Arguments:
             message (Message): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = json.dumps(message)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setMessage_e0781f46", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
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

    def addNode(self):
        """This signal is emitted when a network node was added.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._addNode

    def removeNode(self):
        """This signal is emitted when a network node was removed.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._removeNode

    def syserror(self):
        """This signal is emitted when an error is pushed on the error stack.
           
           Returns: ServiceSignal
           Payload: dict"""
        return self._syserror

    def sysErrorAck(self):
        """This signal is emitted when errors have been acknowledged.
           
           Returns: ServiceSignal"""
        return self._sysErrorAck
