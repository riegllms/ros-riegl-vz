
RICON_UNEXPECTED_MESSAGE = 165
RICON_NO_ROUTE_TO_DESTINATION_NODE = 166
RICON_PROTOCOL_ERROR = 170
RICON_SERVICE_REQUEST_FAILED = 172

_errStrings = {}
_errStrings[RICON_UNEXPECTED_MESSAGE] = "receive unexpected message"
_errStrings[RICON_NO_ROUTE_TO_DESTINATION_NODE] = "no route to destination node"
_errStrings[RICON_PROTOCOL_ERROR] = "riconnect protocol error"
_errStrings[RICON_SERVICE_REQUEST_FAILED] = "service request failed"

def errString(errCode):
    s = _errStrings.get(errCode)
    if s is None:
        s = "ErrorCode: {0}".format(errCode)
    return s
