# pylint: disable=line-too-long
# pylint: disable=too-many-instance-attributes
# pylint: disable=too-many-arguments

import sys
import os
import re
import socket
import struct
import select
import traceback
import weakref
import numbers
import time
from threading import Lock, Thread, Condition
from .lookup3 import hashlittle
from . import ric_pb2
from . import ricerror

Value = ric_pb2.Value

RICON_HASH_INITVAL = 0x31415926
RICON_SWL_NODE_ALIAS_ENDPOINT = "_EP_"
RICON_SWL_NODE_ALIAS_ENDPOINT_HASH = hashlittle(RICON_SWL_NODE_ALIAS_ENDPOINT, initval=RICON_HASH_INITVAL)
RICON_SWL_NODE_ALIAS_SWITCH = "_SW_"
RICON_SWL_NODE_ALIAS_SWITCH_HASH = hashlittle(RICON_SWL_NODE_ALIAS_SWITCH, initval=RICON_HASH_INITVAL)

RICON_SWL_FRAME_TYPE_ANY = 0
RICON_SWL_FRAME_TYPE_DATA = 1
RICON_SWL_FRAME_TYPE_NAMING_REQUEST = 2
RICON_SWL_FRAME_TYPE_NAMING_RESPONSE = 3
RICON_SWL_FRAME_TYPE_DOWNLINK_PORT_REQUEST = 4
RICON_SWL_FRAME_TYPE_DOWNLINK_PORT_RESPONSE = 5
RICON_SWL_FRAME_TYPE_DOWNLINK_PORT_REGISTER = 6
RICON_SWL_FRAME_TYPE_UPLINK_REGISTER = 7
RICON_SWL_FRAME_TYPE_ALIVE_HEARTBEAT = 8
RICON_SWL_FRAME_TYPE_NODE_REMOVE_REQUEST = 9
RICON_SWL_FRAME_TYPE_NO_ROUTE = 10
RICON_SWL_FRAME_TYPE_LOOSE_MESSAGE_SEND = 11
RICON_SWL_FRAME_TYPE_NODE_REMOVE_RESPONSE = 12
RICON_SWL_FRAME_TYPE_ALIVE_HEARTBEAT_ACK = 13

RICON_SWL_PROTOCOL_TYPE_NONE = 0
RICON_SWL_PROTOCOL_TYPE_RIC = 1

RICON_RIC_TRANSFER_MODE_DISABLE = 0
RICON_RIC_TRANSFER_MODE_C2S = 1
RICON_RIC_TRANSFER_MODE_S2C = 2

RICON_RIC_MESSAGE_DOMAIN_ANY = 0
RICON_RIC_MESSAGE_SERVICE_REQUEST = 1
RICON_RIC_MESSAGE_SERVICE_REQUEST_WITH_TRANSFER = 2
RICON_RIC_MESSAGE_SERVICE_RESPONSE = 3
RICON_RIC_MESSAGE_TRANSFER = 4
RICON_RIC_MESSAGE_SIGNAL = 5
RICON_RIC_MESSAGE_SERVICE_RESPONSE_WITH_ERROR = 6
RICON_RIC_MESSAGE_ACK = 127
RICON_RIC_MESSAGE_ACK_REQ = 0x80

RICON_RIC_MESSAGE_TYPE_ANY = 0
RICON_RIC_MESSAGE_TYPE_GET_REQUEST = 1
RICON_RIC_MESSAGE_TYPE_GET_RESPONSE = 2
RICON_RIC_MESSAGE_TYPE_SET_REQUEST = 3
RICON_RIC_MESSAGE_TYPE_SET_RESPONSE = 4
RICON_RIC_MESSAGE_TYPE_GET_ARRAY_REQUEST = 5
RICON_RIC_MESSAGE_TYPE_GET_ARRAY_RESPONSE = 6
RICON_RIC_MESSAGE_TYPE_SET_ARRAY_REQUEST = 7
RICON_RIC_MESSAGE_TYPE_SET_ARRAY_RESPONSE = 8
RICON_RIC_MESSAGE_TYPE_CALL_REQUEST = 9
RICON_RIC_MESSAGE_TYPE_CALL_RESPONSE = 10

RICON_RIC_MESSAGE_TYPE_TRANSFER_ANY = 0
RICON_RIC_MESSAGE_TYPE_TRANSFER_START_REQUEST = 1
RICON_RIC_MESSAGE_TYPE_TRANSFER_START_RESPONSE = 2
RICON_RIC_MESSAGE_TYPE_TRANSFER_BINARY_DATA = 3
RICON_RIC_MESSAGE_TYPE_TRANSFER_ASCII_DATA = 4
RICON_RIC_MESSAGE_TYPE_TRANSFER_PROGRESS_DATA = 5
RICON_RIC_MESSAGE_TYPE_TRANSFER_ERROR_DATA = 6
RICON_RIC_MESSAGE_TYPE_TRANSFER_HELP_DATA = 7
RICON_RIC_MESSAGE_TYPE_TRANSFER_END = 8

RICON_RIC_MESSAGE_TYPE_SIGNAL_ANY = 0
RICON_RIC_MESSAGE_TYPE_SIGNAL_SUBSCRIBE = 1
RICON_RIC_MESSAGE_TYPE_SIGNAL_UNSUBSCRIBE = 2
RICON_RIC_MESSAGE_TYPE_SIGNAL_SEND = 3

CONFIG_NODE_NAME_MAX_STRLEN = 64
CONFIG_NODE_DESC_MAX_STRLEN = 64
CONFIG_RIC_TRANSFER_DATA_BUFFER_SIZE = 32768
CONFIG_RIC_DEFAULT_SERVICE_TIMEOUT_MS = 10000

class RiconnectError(Exception):
    def __init__(self, errCode):
        super().__init__()
        self._errCode = errCode
    def __str__(self):
        return ricerror.errString(self._errCode)

class ServiceError(Exception):
    def __init__(self, errCode, errMsg):
        super().__init__()
        self._errCode = errCode
        self._errMsg = errMsg
    def __str__(self):
        s = "ErrorCode: {0}.".format(self._errCode)
        if self._errMsg:
            s += " {0}".format(self._errMsg)
        return s

class UnknownMethodError(ServiceError):
    def __init__(self):
        super().__init__(-16, "UNKNOWN_METHOD_COMMAND")
    def __str__(self):
        return "Unknown service method"

class _Counter(object):
    def __init__(self):
        self._lck = Lock()
        self._value = 0
    def next(self):
        with self._lck:
            self._value = self._value + 1
            v = self._value
            return v
    def last(self):
        with self._lck:
            return self._value
_linkCounter = _Counter()

def _parseIPv4(address):
    try:
        sp = address.split(':')
        if len(sp) == 2:
            host = sp[0]
            port = int(sp[1])
            return {
                'family': socket.AF_INET,
                'host': host,
                'port': port
            }
    except Exception:
        pass
    return None

def _parseIPv6(address):
    try:
        sp = address.split(':')
        if len(sp) > 3:
            host = ':'.join(sp[0:-1])
            port = int(sp[-1])
            return {
                'family': socket.AF_INET6,
                'host': host,
                'port': port,
                'flowinfo': None,
                'scopeid': None
            }
    except Exception:
        pass
    return None

def _parseAddress(address):
    r = _parseIPv4(address)
    if r is None:
        r = _parseIPv6(address)
    if r is None:
        if sys.platform == "win32":
            if address.find(":") < 0:
                raise RuntimeError("Service address must contain port number.")
        r = {
            'family': socket.AF_UNIX,
            'node': address
        }
    return r

def createFrameHeader(linkId, frameType, srcHash=RICON_SWL_NODE_ALIAS_ENDPOINT_HASH, dstHash=RICON_SWL_NODE_ALIAS_SWITCH_HASH, protocol=RICON_SWL_PROTOCOL_TYPE_NONE):
    frameheader = bytearray(12)
    struct.pack_into("!I", frameheader, 0, srcHash)
    struct.pack_into("!I", frameheader, 4, dstHash)
    struct.pack_into("!H", frameheader, 8, linkId)
    struct.pack_into("B", frameheader, 10, frameType)
    struct.pack_into("B", frameheader, 11, protocol)
    return frameheader

def createMessage(msgDomain, msgType, txnId, msgData=None):
    msg = bytearray(struct.pack("!BBH", msgDomain, msgType, txnId))
    if msgData is not None:
        msg.extend(msgData)
    return msg

def sendFrame(sock, data):
    """Send binary data via socket connection.

       sock ... socket instance
       data ... bytes or bytearray object to send
    """
    datasize = len(data)
    totalsent = 0
    while totalsent < datasize:
        while True:
            try:
                sent = sock.send(data[totalsent:])
                break
            except InterruptedError:
                continue
        if sent == 0:
            raise RuntimeError("socket connection broken")
        totalsent = totalsent + sent

def _receiveData(sock, size):
    data = bytearray()
    while len(data) < size:
        toreceive = size - len(data)
        while True:
            try:
                data.extend(sock.recv(toreceive))
                break
            except InterruptedError:
                continue
        if toreceive == size - len(data):
            # no data received
            time.sleep(0.05)
    return data

def readFrame(sock, timeout=None):
    """Retrieve response frame data from socket connection.

       sock ... socket instance
       timeout ... socket timeout
    """
    timeoutChange = False
    oldTimeout = sock.gettimeout()
    if timeout != oldTimeout:
        sock.settimeout(timeout)
        timeoutChange = True
    frameSize = struct.unpack("!I", _receiveData(sock, 4))[0]
    if frameSize <= 0:
        raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
    frameBuffer = _receiveData(sock, frameSize)
    if timeoutChange:
        sock.settimeout(oldTimeout)
    return frameBuffer

def updateFrameSize(framebuffer):
    struct.pack_into("!I", framebuffer, 0, len(framebuffer)-4)

def readMessage(sock, txnId=None, timeout=None):
    while(True):
        frameBuffer = readFrame(sock, timeout=timeout)
        srcHash, dstHash, linkId, fType, proto = struct.unpack_from(
            "!IIHBB", frameBuffer
        )
        if fType == RICON_SWL_FRAME_TYPE_NO_ROUTE:
            raise RiconnectError(ricerror.RICON_NO_ROUTE_TO_DESTINATION_NODE)
        message = frameBuffer[12:]
        if not message or len(message) < 4:
            return [None] * 4
        msgDomain, msgType, rxnId = struct.unpack_from("!BBH", message)
        msgData = message[4:]
        # check if message needs an acknowledgement
        needsAck = (msgDomain & RICON_RIC_MESSAGE_ACK_REQ) > 0
        if needsAck:
            sendMessageAcknowledgment(sock, linkId, dstHash, srcHash)
            msgDomain = msgDomain & (~RICON_RIC_MESSAGE_ACK_REQ)
        if txnId is None or txnId == rxnId:
            return (msgDomain, msgType, rxnId, msgData)

def sendMessageAcknowledgment(sock, linkId, srcHash, dstHash):
    pkg = bytearray(4)
    pkg.extend(createFrameHeader(linkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=srcHash, dstHash=dstHash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
    pkg.extend(struct.pack("BB", RICON_RIC_MESSAGE_ACK, 0))
    updateFrameSize(pkg)
    sendFrame(sock, pkg)

def readMessageAcknowledgment(sock, txnId=None, timeout=None):
    msgDomain, msgType, rxnId, msgData = readMessage(sock, txnId=txnId, timeout=timeout)
    if (msgDomain & (~RICON_RIC_MESSAGE_ACK_REQ)) != RICON_RIC_MESSAGE_ACK:
        raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)

def sendTransferStart(sock, linkId, txnId, srcHash, dstHash, transferMode, timeout=None):
    pkg = bytearray(4)
    pkg.extend(createFrameHeader(linkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=srcHash, dstHash=dstHash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
    pkg.extend(createMessage(RICON_RIC_MESSAGE_TRANSFER, RICON_RIC_MESSAGE_TYPE_TRANSFER_START_REQUEST, txnId))
    msgData = struct.pack("!BII", transferMode, CONFIG_RIC_TRANSFER_DATA_BUFFER_SIZE, CONFIG_RIC_DEFAULT_SERVICE_TIMEOUT_MS)
    pkg.extend(msgData)
    updateFrameSize(pkg)
    sendFrame(sock, pkg)
    # read response
    msgDomain, msgType, rxnId, msgData = readMessage(sock, txnId=txnId, timeout=timeout)
    if msgDomain != RICON_RIC_MESSAGE_TRANSFER or msgType != RICON_RIC_MESSAGE_TYPE_TRANSFER_START_RESPONSE:
        raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
    remoteTransferMode, remoteBufferSize, remoteTimeoutMs = struct.unpack("!BII", msgData)
    bsize = remoteBufferSize if remoteBufferSize < CONFIG_RIC_TRANSFER_DATA_BUFFER_SIZE else CONFIG_RIC_TRANSFER_DATA_BUFFER_SIZE
    timeout = remoteTimeoutMs if transferMode == RICON_RIC_TRANSFER_MODE_C2S else CONFIG_RIC_DEFAULT_SERVICE_TIMEOUT_MS
    return (bsize, timeout)

def sendTransferData(sock, linkId, txnId, srcHash, dstHash, data, bufferSize, timeout=None):
    dataLength = len(data)
    dataSent = 0
    while dataSent < dataLength:
        pkgDataSize = dataLength - dataSent
        if pkgDataSize > bufferSize:
            pkgDataSize = bufferSize
        pkgData = data[dataSent:dataSent+pkgDataSize]
        pkg = bytearray(4)
        pkg.extend(createFrameHeader(linkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=srcHash, dstHash=dstHash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
        pkg.extend(createMessage(RICON_RIC_MESSAGE_TRANSFER | RICON_RIC_MESSAGE_ACK_REQ, RICON_RIC_MESSAGE_TYPE_TRANSFER_BINARY_DATA, txnId))
        pkg.extend(pkgData)
        updateFrameSize(pkg)
        sendFrame(sock, pkg)
        readMessageAcknowledgment(sock, txnId=txnId, timeout=timeout)
        dataSent = dataSent + pkgDataSize

def sendTransferEnd(sock, linkId, txnId, srcHash, dstHash, timeout=None):
    pkg = bytearray(4)
    pkg.extend(createFrameHeader(linkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=srcHash, dstHash=dstHash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
    pkg.extend(createMessage(RICON_RIC_MESSAGE_TRANSFER | RICON_RIC_MESSAGE_ACK_REQ, RICON_RIC_MESSAGE_TYPE_TRANSFER_END, txnId))
    updateFrameSize(pkg)
    sendFrame(sock, pkg)
    # read response
    readMessageAcknowledgment(sock, txnId=txnId, timeout=timeout)

def sendNodeRemoveRequest(sock, linkId, srcHash):
    pkg = bytearray(4)
    pkg.extend(createFrameHeader(linkId, RICON_SWL_FRAME_TYPE_NODE_REMOVE_REQUEST, srcHash=srcHash))
    updateFrameSize(pkg)
    sendFrame(sock, pkg)
    # read response
    frameBuffer = readFrame(sock, timeout=5)
    fType = struct.unpack_from("!IIHBB", frameBuffer)[3]
    if fType != RICON_SWL_FRAME_TYPE_NODE_REMOVE_RESPONSE:
        raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)

class Messenger(object):
    """The messenger allows to push an new message on the riconnect message stack."""
    def __init__(self):
        self._socket = None
        self._linkId = 0

    def __del__(self):
        self.close()

    def open(self, address):
        """Open connection to riconnect switch.

           Arguments:
             address (str): address consisting of hostname/IP-address and port number
                            e.g. 127.0.0.1:20000"""
        if self._socket is None:
            self._linkId = _linkCounter.next()
            endpoint = _parseAddress(address)
            self._socket = socket.socket(family=endpoint['family'], type=socket.SOCK_STREAM)
            if endpoint['family'] == socket.AF_INET:
                self._socket.connect((endpoint['host'], endpoint['port']))
            else:
                self._socket.connect((endpoint['node']))

    def close(self):
        """Close connection."""
        if self._socket is not None:
            self._socket.close()

    def send(self, category, name, data):
        """Send message.

           Arguments:
             category (str): message category
             name (str): message name
             data (str)
             data (bytes): message data"""
        # create request frame
        pkg = bytearray(4)
        # add frame header
        srcHash = hashlittle("", initval=RICON_HASH_INITVAL)
        pkg.extend(createFrameHeader(self._linkId, RICON_SWL_FRAME_TYPE_LOOSE_MESSAGE_SEND, srcHash=srcHash))
        # add message name
        pkg.extend(name.encode(encoding="utf-8"))
        pkg.extend(b'\0') # name must be 0 terminated
        # add message category
        pkg.extend(category.encode(encoding="utf-8"))
        pkg.extend(b'\0') # category must be 0 terminated
        # add message data
        if isinstance(data, bytes) or isinstance(data, bytearray):
            # add message length
            pkg.extend(struct.pack("!I", len(data)))
            pkg.extend(data)
        else:
            encodedData = data.encode(encoding="utf-8")
            pkg.extend(struct.pack("!I", len(encodedData)))
            pkg.extend(encodedData)
        updateFrameSize(pkg)
        sendFrame(self._socket, pkg)

class HeartbeatThread(Thread):
    """The service heartbeat thread."""
    def __init__(self, nodeName, nodeNameHash, sock, linkId, interval=5):
        super().__init__(name="{0}-heartbeat-thread".format(nodeName))
        self._nodeNameHash = nodeNameHash
        self._socket = sock
        self._linkId = linkId
        self._interval = interval
        self._stopped = False
        self._lock = Lock()
        self._wc = Condition(lock=self._lock)
        self._connected = False

    def run(self):
        with self._lock:
            self._connected = True
            self._socket.settimeout(3)
            while not self._stopped:
                try:
                    self._sendHeartbeat()
                except Exception:
                    self._stopped = True
                if not self._stopped:
                    self._wc.wait(timeout=self._interval)
            self._connected = False

    def stop(self):
        with self._lock:
            self._stopped = True
            self._wc.notify()

    def is_connected(self):
        with self._lock:
            return self._connected

    def _sendHeartbeat(self):
        pkg = bytearray(4)
        pkg.extend(createFrameHeader(self._linkId, RICON_SWL_FRAME_TYPE_ALIVE_HEARTBEAT, srcHash=self._nodeNameHash))
        pkg.extend(struct.pack("B", 10)) # interval + check timeout in seconds
        updateFrameSize(pkg)
        sendFrame(self._socket, pkg)
        frameBuffer = readFrame(self._socket)
        srcHash, dstHash, linkId, fType, proto = struct.unpack_from(
            "!IIHBB", frameBuffer
        )
        if not (
                (srcHash == RICON_SWL_NODE_ALIAS_SWITCH_HASH) and
                (dstHash == self._nodeNameHash) and
                (linkId == self._linkId) and
                (fType == RICON_SWL_FRAME_TYPE_ALIVE_HEARTBEAT_ACK) and
                (proto == RICON_SWL_PROTOCOL_TYPE_NONE)
            ):
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)

class SignalThread(Thread):
    """The service signal thread."""
    def __init__(self, nodeName, nodeNameHash, sock, linkId, signalFunc):
        super().__init__(name="{0}-signal-thread".format(nodeName))
        self._nodeNameHash = nodeNameHash
        self._socket = sock
        self._linkId = linkId
        self._stopped = False
        self._lock = Lock()
        self._signalFunc = signalFunc

    def run(self):
        while True:
            with self._lock:
                if self._stopped:
                    break
            try:
                self._receiveData()
            except Exception:
                stopped = False
                with self._lock:
                    stopped = self._stopped
                if not stopped:
                    traceback.print_exc()

    def stop(self):
        with self._lock:
            self._stopped = True

    def _receiveData(self):
        rlist, wlist, xlist = select.select([self._socket], [], [], 0.5)
        if not rlist:
            # timeout
            return
        # read whole message
        msgDomain, msgType, rxnId, msgData = readMessage(self._socket)
        # check if message is a signal
        if msgDomain == RICON_RIC_MESSAGE_SIGNAL and msgType == RICON_RIC_MESSAGE_TYPE_SIGNAL_SEND:
            # decode message SignalSend
            sigData = ric_pb2.SignalSend()
            sigData.ParseFromString(bytes(msgData))
            if self._signalFunc:
                stopped = False
                with self._lock:
                    stopped = self._stopped
                if not stopped:
                    self._signalFunc(sigData)

class ConnectionFactory(object):
    """Connection factory for socket connections."""
    def __init__(self):
        self._lock = Lock()
        self._rootSockets = {}
        self._serviceSockets = {}
        self._signalSockets = {}
    def acquireRootConnection(self, address):
        with self._lock:
            return self._acquireConnection(self._rootSockets, address)
    def acquireServiceConnection(self, address):
        with self._lock:
            return self._acquireConnection(self._serviceSockets, address)
    def acquireSignalConnection(self, address):
        with self._lock:
            return self._acquireConnection(self._signalSockets, address)
    def releaseRootConnection(self, address, srcHash):
        with self._lock:
            return self._releaseConnection(self._rootSockets, address, srcHash)
    def releaseServiceConnection(self, address):
        with self._lock:
            return self._releaseConnection(self._serviceSockets, address)
    def releaseSignalConnection(self, address):
        with self._lock:
            return self._releaseConnection(self._signalSockets, address)
    def _acquireConnection(self, cache, address):
        r = cache.get(address)
        if r is None:
            linkId = _linkCounter.next()
            endpoint = _parseAddress(address)
            if endpoint['family'] == socket.AF_INET:
                addr = (endpoint['host'], endpoint['port'])
                sock = socket.create_connection(addr, 20)
                sock.settimeout(None)
            elif endpoint['family'] == socket.AF_INET6:
                addr = (endpoint['host'], endpoint['port'], endpoint['flowinfo'], endpoint['scopeid'])
                sock = socket.create_connection(addr, 20)
                sock.settimeout(None)
            else:
                sock = socket.socket(family=endpoint['family'], type=socket.SOCK_STREAM)
                sock.settimeout(20)
                sock.connect((endpoint['node']))
                sock.settimeout(None)
            r = [sock, linkId, 1]
            cache[address] = r
            return r[:2]
        else:
            r[2] += 1
        return r[:2]
    def _releaseConnection(self, cache, address, srcHash=None):
        r = cache.get(address)
        if r is not None:
            if r[2] > 1:
                r[2] -= 1
            else:
                # close connection
                sock = r[0]
                try:
                    if srcHash is not None:
                        sendNodeRemoveRequest(sock, r[1], srcHash)
                    sock.shutdown(socket.SHUT_RDWR)
                    sock.close()
                except:
                    pass
                del cache[address]

class Service(object):
    """The riconnect Service class allows communication with Riegl's VZi scanners.

       Example:
         svc = riconnect.Service("SCANNER")
         svc.open("127.0.0.1:20000")
         print("Indentifier: " + svc.getProperty("INST_IDENT").s)
         svc.close()"""
    def __init__(self, name, connectionFactory=None):
        """Create new service instance.

           Arguments:
             name (str): service name"""
        self._factory = ConnectionFactory() if connectionFactory is None else connectionFactory
        self._servicename = name
        self._servicenameHash = hashlittle(name, initval=RICON_HASH_INITVAL)
        self._serviceLock = Lock()
        self._rootConnection = None
        self._rootConnectionLinkId = 0
        self._serviceConnection = None
        self._serviceConnectionLinkId = 0
        self._signalConnection = None
        self._signalConnectionLinkId = 0
        self._client_name = name + "-client"
        self._client_hash = hashlittle(self._client_name, initval=RICON_HASH_INITVAL)
        self._subscriptions = {}
        # use program name as node description
        self._node_description = "DefaultNodeDesc"
        try:
            if sys.argv:
                self._node_description = os.path.splitext(os.path.basename(sys.argv[0]))[0]
        except Exception:
            self._node_description = "DefaultNodeDesc"
        self._heartbeatthread = None
        self._signalthread = None
        self._txnId = 0
        self._address = None

    def open(self, address):
        """Open service connection.

           Arguments:
             address (str): service address consisting of hostname/IP-address and port number
                            e.g. 127.0.0.1:20000
        """
        with self._serviceLock:
            if self._address is not None:
                return
            self._rootConnection, self._rootConnectionLinkId = self._factory.acquireRootConnection(address)
            self._serviceConnection, self._serviceConnectionLinkId = self._factory.acquireServiceConnection(address)
            self._signalConnection, self._signalConnectionLinkId = self._factory.acquireSignalConnection(address)
            self._address = address
            self._sendNameRequest()
            self._heartbeatthread = HeartbeatThread(self._client_name, self._client_hash, self._rootConnection, self._rootConnectionLinkId)
            self._heartbeatthread.daemon = True
            self._heartbeatthread.start()
            self._signalthread = SignalThread(self._client_name, self._client_hash, self._signalConnection, self._signalConnectionLinkId, self._onSignalReceived)
            self._signalthread.daemon = True
            self._signalthread.start()

    def close(self):
        """Close service connection."""
        with self._serviceLock:
            if self._address is not None:
                self._subscriptions = {}
                if self._signalthread is not None:
                    self._signalthread.stop()
                self._factory.releaseSignalConnection(self._address)
                self._signalConnection = None
                self._factory.releaseServiceConnection(self._address)
                self._serviceConnection = None
                if self._heartbeatthread is not None:
                    self._heartbeatthread.stop()
                self._factory.releaseRootConnection(self._address, self._client_hash)
                self._rootConnection = None
                self._address = None

    def _sendNameRequest(self):
        nodeName = bytearray(self._client_name.encode(encoding="utf-8"))
        if len(nodeName) >= CONFIG_NODE_NAME_MAX_STRLEN:
            nodeName = nodeName[:CONFIG_NODE_NAME_MAX_STRLEN-1]
        nodeName = nodeName.ljust(CONFIG_NODE_NAME_MAX_STRLEN, b'\0')
        nodeDesc = bytearray(self._node_description.encode(encoding="utf-8"))
        if len(nodeDesc) >= CONFIG_NODE_DESC_MAX_STRLEN:
            nodeDesc = nodeDesc[:CONFIG_NODE_DESC_MAX_STRLEN-1]
        nodeDesc = nodeDesc.ljust(CONFIG_NODE_DESC_MAX_STRLEN, b'\0')
        # create data package
        pkg = bytearray(4)
        pkg.extend(createFrameHeader(self._rootConnectionLinkId, RICON_SWL_FRAME_TYPE_NAMING_REQUEST))
        pkg.extend(struct.pack("B", 0)) # force
        pkg.extend(nodeName)
        pkg.extend(nodeDesc)
        updateFrameSize(pkg)
        sendFrame(self._rootConnection, pkg)
        # get result
        frameBuffer = readFrame(self._rootConnection, timeout=10)
        if self._checkFrame(frameBuffer, RICON_SWL_FRAME_TYPE_NAMING_RESPONSE):
            recNodeName = frameBuffer[12:].rstrip(b'\0').decode(encoding="utf-8")
            self._client_name = recNodeName
            self._client_hash = hashlittle(self._client_name, initval=RICON_HASH_INITVAL)

    def _checkFrame(self, frameBuffer, frameType):
        srcHash, dstHash, linkId, fType, proto = struct.unpack_from(
            "!IIHBB", frameBuffer
        )
        return (
            (srcHash == RICON_SWL_NODE_ALIAS_SWITCH_HASH) and
            (dstHash == RICON_SWL_NODE_ALIAS_ENDPOINT_HASH) and
            (linkId == self._rootConnectionLinkId) and
            (fType == frameType) and
            (proto == RICON_SWL_PROTOCOL_TYPE_NONE)
        )

    def subscribe(self, signalName, cbFunc):
        """Subscribe to service signal.

           The specified callback function must have one position argument
           which is the payload (bytes) of the signal.

           Arguments:
             signalName (str): name of service signal
             cbFunc (function): callback function with one argument"""
        sigHash = hashlittle(signalName, initval=RICON_HASH_INITVAL)
        with self._serviceLock:
            if self._rootConnection is None:
                raise RuntimeError("Service connection not established.")
            subscribers = self._subscriptions.get(sigHash)
            if subscribers is None:
                subscribers = []
                self._subscriptions[sigHash] = subscribers
            if len(subscribers) == 0:
                self._sendSignalSubscribe(signalName)
            subscribers.append(cbFunc)

    def unsubscribe(self, signalName, cbFunc):
        """Unsubscribe from service signal.

           Arguments:
             signalName (str): name of service signal
             cbFunc (function): callback function with one argument"""
        sigHash = hashlittle(signalName, initval=RICON_HASH_INITVAL)
        with self._serviceLock:
            subscribers = self._subscriptions.get(sigHash)
            if (subscribers is not None) and (cbFunc in subscribers):
                subscribers.remove(cbFunc)
                if len(subscribers) == 0:
                    self._sendSignalUnsubscribe(signalName)

    def _onSignalReceived(self, sigData):
        if sigData.signaller != self._servicenameHash:
            return
        if sigData.subscriber != self._client_hash:
            return
        with self._serviceLock:
            subscribers = self._subscriptions.get(sigData.signal, [])[:]
        for sub in subscribers:
            try:
                if isinstance(sub, weakref.WeakMethod):
                    fn = sub()
                    if fn is not None:
                        fn(sigData.data)
                else:
                    sub(sigData.data)
            except Exception:
                pass

    def _sendSignalSubscribe(self, signalName):
        pkg = bytearray(4)
        pkg.extend(createFrameHeader(self._signalConnectionLinkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=self._client_hash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
        payload = ric_pb2.SignalSubscribe()
        payload.signal = signalName
        payload.signaller = self._servicename
        payload.subscriber = self._client_name
        msgData = payload.SerializeToString()
        pkg.extend(createMessage(RICON_RIC_MESSAGE_SIGNAL, RICON_RIC_MESSAGE_TYPE_SIGNAL_SUBSCRIBE, 0, msgData))
        updateFrameSize(pkg)
        sendFrame(self._signalConnection, pkg)

    def _sendSignalUnsubscribe(self, signalName):
        pkg = bytearray(4)
        pkg.extend(createFrameHeader(self._signalConnectionLinkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=self._client_hash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
        payload = ric_pb2.SignalUnsubscribe()
        payload.signal = signalName
        payload.signaller = self._servicename
        payload.subscriber = self._client_name
        msgData = payload.SerializeToString()
        pkg.extend(createMessage(RICON_RIC_MESSAGE_SIGNAL, RICON_RIC_MESSAGE_TYPE_SIGNAL_UNSUBSCRIBE, 0, msgData))
        updateFrameSize(pkg)
        sendFrame(self._signalConnection, pkg)

    def callFunction(self, name, inputs=None, inputTransfers=None, numOutputTransfers=0, timeout=-1):
        """Call service function.

           Arguments:
             name (str): function name
             inputs (list(Value)): function arguments
             inputTransfers (list(bytes)): function transfer buffers
             numOutputTransfers (int): number of output transfer buffers returned by the service function
             timeout (float): timeout in seconds a data request is allowed to take. Please note that
                              this timeout is not the timeout of the whole function but each request
                              made in the function. If a timeout of 5 seconds is specified and this
                              function makes two data requests to the server with each request taking
                              4 seconds then this call will not time out even though it took 8 seconds.
                              If not timeout is specified or the timeout is smaller than 1 then the
                              default timeout is used. None can be used to disable all request timeouts."""
        self._checkInputs(inputs)
        con, linkId, txnId = (None, None, None)
        with self._serviceLock:
            con = self._serviceConnection
            linkId = self._serviceConnectionLinkId
            self._txnId = ( self._txnId + 1 ) % (2**16)
            txnId = self._txnId
        if con is None:
            raise RuntimeError("Service connection not established.")
        if timeout is not None and not isinstance(timeout, numbers.Number):
            raise TypeError("Timeout must be numer.")
        if timeout is not None and timeout < 1:
            timeout =  CONFIG_RIC_DEFAULT_SERVICE_TIMEOUT_MS / 1000
        request = ric_pb2.CallRequest()
        request.command = name
        request.num = 0
        if inputs is not None:
            request.num = len(inputs)
            for v in inputs:
                request.param.add().CopyFrom(v)
        # create data package
        requestMsgDomain = RICON_RIC_MESSAGE_SERVICE_REQUEST
        if inputTransfers is not None or numOutputTransfers > 0:
            requestMsgDomain = RICON_RIC_MESSAGE_SERVICE_REQUEST_WITH_TRANSFER
        pkg = bytearray(4)
        pkg.extend(createFrameHeader(linkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=self._client_hash, dstHash=self._servicenameHash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
        pkg.extend(createMessage(requestMsgDomain, RICON_RIC_MESSAGE_TYPE_CALL_REQUEST, txnId))
        pkg.extend(request.SerializeToString())
        updateFrameSize(pkg)
        # send request
        sendFrame(self._serviceConnection, pkg)
        if inputTransfers is not None:
            self._sendRequestTransferBuffers(con, linkId, txnId, inputTransfers, timeout=timeout)
        # wait for response
        outputs = []
        outputTransfers = []
        if numOutputTransfers > 0:
            outputTransfers = self._readResponseTransferBuffers(con, linkId, txnId, timeout=timeout)
            if numOutputTransfers == 1 and len(outputTransfers) > 1:
                b = bytearray()
                for ot in outputTransfers:
                    b.extend(ot)
                outputTransfers = [bytes(b)]
            if len(outputTransfers) > numOutputTransfers:
                outputTransfers = outputTransfers[0:numOutputTransfers]
            if len(outputTransfers) < numOutputTransfers:
                outputTransfers.extend([bytes()]*(numOutputTransfers-len(outputTransfers)))
        msgDomain, msgType, rxnId, msgData = readMessage(con, txnId=txnId, timeout=timeout)
        # validate response
        if not msgDomain in [RICON_RIC_MESSAGE_SERVICE_RESPONSE, RICON_RIC_MESSAGE_SERVICE_RESPONSE_WITH_ERROR]:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if not msgType == RICON_RIC_MESSAGE_TYPE_CALL_RESPONSE:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if rxnId != txnId:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if msgData is not None:
            response = ric_pb2.CallResponse()
            response.ParseFromString(bytes(msgData))
            if response.num > 0:
                for i in range(0, response.num):
                    outputs.append(response.param[i])
            status = response.status
            errMsg = response.errstr if response.HasField('errstr') else None
            if status != 0:
                if errMsg == "UNKNOWN_METHOD_COMMAND":
                    raise UnknownMethodError()
                else:
                    raise ServiceError(status, errMsg)
        return (outputs, outputTransfers)

    def getProperty(self, name, index=None, timeout=None):
        """Get value of service property.

           Arguments:
             name (str): property name
             index (int)
             index (list(int)): property index (needed for arrays and matrices)
             timeout (float): see callFunction()"""
        # parse property indices
        idx0, idx1, idx2 = [0, 0, 0]
        if index is not None:
            if isinstance(index, int):
                idx0 = index
            elif isinstance(index, list) or isinstance(index, tuple):
                indexLen = len(index)
                idx0 = int(index[0] if indexLen > 0 else 0)
                idx1 = int(index[1] if indexLen > 1 else 0)
                idx2 = int(index[2] if indexLen > 2 else 0)
                if idx0 < 0 or idx1 < 0 or idx2 < 0:
                    raise ValueError("Index must be positive.")
            else:
                raise TypeError("Index must be an int of list of ints.")
        if timeout is not None and not isinstance(timeout, numbers.Number):
            raise TypeError("Timeout must be numer.")
        if timeout is not None and timeout < 1:
            timeout =  CONFIG_RIC_DEFAULT_SERVICE_TIMEOUT_MS / 1000
        con, linkId, txnId = (None, None, None)
        with self._serviceLock:
            con = self._serviceConnection
            linkId = self._serviceConnectionLinkId
            self._txnId = ( self._txnId + 1 ) % (2**16)
            txnId = self._txnId
        if con is None:
            raise RuntimeError("Service connection not established.")
        request = ric_pb2.GetRequest()
        request.command = name
        request.index0 = idx0
        request.index1 = idx1
        request.index2 = idx2
        # create data package
        requestMsgDomain = RICON_RIC_MESSAGE_SERVICE_REQUEST
        pkg = bytearray(4)
        pkg.extend(createFrameHeader(linkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=self._client_hash, dstHash=self._servicenameHash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
        pkg.extend(createMessage(requestMsgDomain, RICON_RIC_MESSAGE_TYPE_GET_REQUEST, txnId))
        pkg.extend(request.SerializeToString())
        updateFrameSize(pkg)
        # send request
        sendFrame(self._serviceConnection, pkg)
        # wait for response
        output = None
        msgDomain, msgType, rxnId, msgData = readMessage(con, txnId=txnId, timeout=timeout)
        # validate response
        if not msgDomain in [RICON_RIC_MESSAGE_SERVICE_RESPONSE, RICON_RIC_MESSAGE_SERVICE_RESPONSE_WITH_ERROR]:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if not msgType == RICON_RIC_MESSAGE_TYPE_GET_RESPONSE:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if rxnId != txnId:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if msgData is not None:
            response = ric_pb2.GetResponse()
            response.ParseFromString(bytes(msgData))
            status = response.status
            errMsg = response.errstr if response.HasField('errstr') else None
            if status != 0:
                raise ServiceError(status, errMsg)
            output = response.value
        return output

    def setProperty(self, name, value, index=None, timeout=None):
        """Set value of service property.

           Arguments:
             name (str): property name
             value (Value): new property value
             index (int)
             index (list(int)): property index (needed for arrays and matrices)
             timeout (float): see callFunction()"""
        if not isinstance(value, Value):
            raise TypeError("Value must be a Value object.")
        # parse property indices
        idx0, idx1, idx2 = [0, 0, 0]
        if index is not None:
            if isinstance(index, int):
                idx0 = index
            elif isinstance(index, list) or isinstance(index, tuple):
                indexLen = len(index)
                idx0 = int(index[0] if indexLen > 0 else 0)
                idx1 = int(index[1] if indexLen > 1 else 0)
                idx2 = int(index[2] if indexLen > 2 else 0)
                if idx0 < 0 or idx1 < 0 or idx2 < 0:
                    raise ValueError("Index must be positive.")
            else:
                raise TypeError("Index must be an int of list of ints.")
        if timeout is not None and not isinstance(timeout, numbers.Number):
            raise TypeError("Timeout must be numer.")
        if timeout is not None and timeout < 1:
            timeout =  CONFIG_RIC_DEFAULT_SERVICE_TIMEOUT_MS / 1000
        con, linkId, txnId = (None, None, None)
        with self._serviceLock:
            con = self._serviceConnection
            linkId = self._serviceConnectionLinkId
            self._txnId = ( self._txnId + 1 ) % (2**16)
            txnId = self._txnId
        if con is None:
            raise RuntimeError("Service connection not established.")
        request = ric_pb2.SetRequest()
        request.command = name
        request.index0 = idx0
        request.index1 = idx1
        request.index2 = idx2
        request.value.CopyFrom(value)
        # create data package
        requestMsgDomain = RICON_RIC_MESSAGE_SERVICE_REQUEST
        pkg = bytearray(4)
        pkg.extend(createFrameHeader(linkId, RICON_SWL_FRAME_TYPE_DATA, srcHash=self._client_hash, dstHash=self._servicenameHash, protocol=RICON_SWL_PROTOCOL_TYPE_RIC))
        pkg.extend(createMessage(requestMsgDomain, RICON_RIC_MESSAGE_TYPE_SET_REQUEST, txnId))
        pkg.extend(request.SerializeToString())
        updateFrameSize(pkg)
        # send request
        sendFrame(self._serviceConnection, pkg)
        # wait for response
        msgDomain, msgType, rxnId, msgData = readMessage(con, txnId=txnId, timeout=timeout)
        # validate response
        if not msgDomain in [RICON_RIC_MESSAGE_SERVICE_RESPONSE, RICON_RIC_MESSAGE_SERVICE_RESPONSE_WITH_ERROR]:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if not msgType == RICON_RIC_MESSAGE_TYPE_SET_RESPONSE:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if rxnId != txnId:
            raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
        if msgData is not None:
            response = ric_pb2.SetResponse()
            response.ParseFromString(bytes(msgData))
            status = response.status
            errMsg = response.errstr if response.HasField('errstr') else None
            if status != 0:
                raise ServiceError(status, errMsg)

    def _checkInputs(self, inputs):
        if inputs is None:
            return
        if not isinstance(inputs, list):
            raise RuntimeError("Inputs must be a list of Value objects.")
        for i in inputs:
            if not isinstance(i, Value):
                raise RuntimeError("Inputs must be a list of Value objects.")

    def _sendRequestTransferBuffers(self, sock, linkId, txnId, buffers, timeout=None):
        srcHash = self._client_hash
        dstHash = self._servicenameHash
        # begin transfer buffer data exchange
        bufferSize = sendTransferStart(
            sock, linkId, txnId, srcHash, dstHash, RICON_RIC_TRANSFER_MODE_C2S,
            timeout=timeout
        )[0]
        # send transfer buffer data
        for databuffer in buffers:
            sendTransferData(
                sock, linkId, txnId, srcHash, dstHash, databuffer, bufferSize,
                timeout=timeout
            )
        # end transfer buffer data exchange
        sendTransferEnd(sock, linkId, txnId, srcHash, dstHash, timeout=timeout)

    def _readResponseTransferBuffers(self, sock, linkId, txnId, progressHandler=None, timeout=None):
        dataBuffers = []
        responseTransferEnd = False
        srcHash = self._client_hash
        dstHash = self._servicenameHash
        # begin transfer buffer data exchange
        sendTransferStart(
            sock, linkId, txnId, srcHash, dstHash, RICON_RIC_TRANSFER_MODE_S2C,
            timeout=timeout
        )
        # read transfer buffer packages
        while not responseTransferEnd:
            msgDomain, msgType, rxnId, msgData = readMessage(sock, txnId=txnId, timeout=timeout)
            if msgDomain != RICON_RIC_MESSAGE_TRANSFER:
                raise RiconnectError(ricerror.RICON_UNEXPECTED_MESSAGE)
            if msgType == RICON_RIC_MESSAGE_TYPE_TRANSFER_END:
                responseTransferEnd = True
            elif msgType in (RICON_RIC_MESSAGE_TYPE_TRANSFER_BINARY_DATA, RICON_RIC_MESSAGE_TYPE_TRANSFER_ASCII_DATA):
                dataBuffers.append(msgData)
            elif msgType == RICON_RIC_MESSAGE_TYPE_TRANSFER_PROGRESS_DATA:
                if len(msgData) == 4 and progressHandler is not None:
                    prog = struct.unpack("!f", msgData)[0]
                    progressHandler(prog)

        return dataBuffers
