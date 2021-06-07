# pylint: skip-file
import json
import struct
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


def _geosysservice_error_decoder(payload):
    return json.loads(payload.decode())

class GeoSysService(object):
    """The geosys service.
       
       The geosys service provides access to the GeoSysManager database of the
       scanner. Each scanner contains a default GeoSys database containing two
       coordinate system. Additionally to the default database, a user can copy a
       custom GeoSys database to system which will replace the default database.
       The GeoSysService can be used to retrieve information about the available
       coordinate systems and lets you transform a set of coordinates from one
       coordinate system to another.
       
       Axis Types:
          0 ... Invalid axis
          1 ... Easting
          2 ... Northing
          3 ... Westing
          4 ... Southing
          5 ... Height
          6 ... Latitude
          7 ... Longitude
          8 ... Altitude
          9 ... X
         10 ... Y
         11 ... Z
       
       Coordinate System Types:
         GEOCENTRIC ... X, Y, Z
         GEOGRAPHIC ... Latitude, Longitude, Altitude
         PROJECTION ... one of (Easting, Northing, Westing, Southing) for first
                        two axes and Heightfor last axis
         UNKNOWN    ... unknown axes configuration"""

    def __init__(self, address):
        self._svc = riconnect.Service("GeoSysService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_geosysservice_error_decoder)
        self._coordinateSystemsChanged = ServiceSignal(self._svc, "coordinateSystemsChanged")
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def coordinateSystems(self):
        """Return the available coordinate systems.
           
           Returns a JSON string in the followring format:
           [
             {
               "uid": "",
               "name": "",
               "epsgCode": 1234,
               "comments": "",
               "type": "",
               "axisName1": "",
               "axisName2": "",
               "axisName3": "",
               "axisUnit1": "",
               "axisUnit2": "",
               "axisUnit3": "",
               "axisType1": 0,
               "axisType2": 0,
               "axisType3": 0
             },
             ...
           ]
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("coordinateSystems_5b748add", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def transformFromWgs84(self, latitude, longitude, height, targetUID):
        """Transform coordinates from WGS84 to some other coordinate system.
           
           Returns JSON object in the followring format.
           {
             "coord1": 0,
             "coord2": 0,
             "coord3": 0,
             "coordString1": "0 0' 0''",
             "coordString2": "0 0' 0''",
             "coordString3": "0 0' 0''",
           }
           
           Arguments:
             latitude (float): the coordinate latitude
             longitude (float): the coordinate longitude
             height (float): the coordinate height (altitude)
             targetUID (str): the GeoSysManager UID of the target coordinate system
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].d = latitude
        inputs[1].d = longitude
        inputs[2].d = height
        inputs[3].s = targetUID
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("transformFromWgs84_e151868a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def transformToWgs84(self, sourceUID, coord1, coord2, coord3):
        """Transform coordinates from a coordinate system to WGS84.
           
           Returns JSON object in the followring format.
           {
             "latitude": 0,
             "longitude": 0,
             "height": 0,
             "latitudeString": "0 0' 0''",
             "longitudeString: "0 0' 0''"
           }
           
           Arguments:
             sourceUID (str): the GeoSysManager UID of the coordinate system
             coord1 (float): the coordinate of the first axis
             coord2 (float): the coordinate of the second axis
             coord3 (float): the coordinate of the third axis
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].s = sourceUID
        inputs[1].d = coord1
        inputs[2].d = coord2
        inputs[3].d = coord3
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("transformToWgs84_1bf22b2b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def transformCoordinate(self, sourceUID, targetUID, coord1, coord2, coord3):
        """Transform coordinates from one coordinate system to another.
           
           Returns JSON object in the followring format.
           {
             "coord1": 0,
             "coord2": 0,
             "coord3": 0,
             "coordString1": "0 0' 0''",
             "coordString2": "0 0' 0''",
             "coordString3": "0 0' 0''",
           }
           
           Arguments:
             sourceUID (str): the GeoSysManager UID of the source coordinate system
             targetUID (str): the GeoSysManager UID of the target coordinate system
             coord1 (float): the coordinate of the first axis
             coord2 (float): the coordinate of the second axis
             coord3 (float): the coordinate of the third axis
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 5)]
        inputs[0].s = sourceUID
        inputs[1].s = targetUID
        inputs[2].d = coord1
        inputs[3].d = coord2
        inputs[4].d = coord3
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("transformCoordinate_6edf67a8", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def verifyCoordinates(self, UID, coord1, coord2, coord3):
        """Check the coordinate for validity.
           
           Returns a validity for each coordinate. The possible    values are:
             0 ... coordinate is valid
             1 ... coordinate is invalid (e.g. not a number)
             2 ... coordinate is out of range
           
           Arguments:
             UID (str): the GeoSysManager UID of the coordinate system
             coord1 (float): first coordinate
             coord2 (float): second coordinate
             coord3 (float): third coordinate
           
           Returns: tuple(int, int, int)
             1: validity of first coordinate
             2: validity of seconds coordinate
             3: validity of third coordinate"""
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].s = UID
        inputs[1].d = coord1
        inputs[2].d = coord2
        inputs[3].d = coord3
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("verifyCoordinates_0811caf3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []
        outputs.append(rvalues[0].u32)
        outputs.append(rvalues[1].u32)
        outputs.append(rvalues[2].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def findCoordinateSystem(self, ident):
        """Find a GeosysManager coordinate system.
           
           Find a coordinate system based on name, EPSG code, or UID.
           The following strategy is used to find the coordinate system:
               1. Identifier starts with 'EPSG::' followed by the EPSG code.
                  Compare with EPSG code.
               2. Compare with UID.
               3. Compare with name.
           
           Returns a JSON string in the followring format:
           {
             "uid": "",
             "name": "",
             "epsgCode": 1234,
             "comments": "",
             "type": "",
             "axisName1": "",
             "axisName2": "",
             "axisName3": "",
             "axisUnit1": "",
             "axisUnit2": "",
             "axisUnit3": "",
             "axisType1": 0,
             "axisType2": 0,
             "axisType3": 0
           }
           
           If no coordinate system was found then an empty JSON object {} is returned.
           
           Arguments:
             ident (str): corrdinate system identifier (UID, name, EPSG)
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = ident
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("findCoordinateSystem_d3ec3a13", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def reloadCoordinateSystems(self):
        """Reload the coordinate systems from the GeoSysManager database."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("reloadCoordinateSystems_f3f4bc7f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def activeDatabase(self):
        """Return the file path of the active GeoSysManager database.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("activeDatabase_020aea13", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def datumTransformationRequired(self, source, destination):
        """Return true if a datum transformation is required between the two
           specified coordinate systems.
           
           New in version 1.3
           
           Arguments:
             source (str): identifier of source coordinate system
             destination (str): identifier of target coordinate system
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = source
        inputs[1].s = destination
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("datumTransformationRequired_8dbfbed3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def findGeocentricCoordinateSystem(self, *args):
        """
           findGeocentricCoordinateSystem(csID)
               Find a geocentric coordinate system that has the best match with the specified coordinate system.
               
               In most cases this returns the first geocentric coordinate system where no
               datum transformation is required. The returned string is the coordinate system's
               GeoSysManager UID. If no coordinate system could be found then an empty string
               is returned.
               
               New in version 1.3
               
               Arguments:
                 csID (str): identifier of coordinate system
               
               Returns: str

           findGeocentricCoordinateSystem(csID, forceXYZ)
               Find a geocentric coordinate system that has the best match with the specified coordinate system.
               
               In most cases this returns the first geocentric coordinate system where no
               datum transformation is required. The returned string is the coordinate system's
               GeoSysManager UID. If no coordinate system could be found then an empty string
               is returned.
               
               New in version 1.3
               
               Arguments:
                 csID (str): identifier of coordinate system
                 forceXYZ (bool): if true then the geocentric coordinate system must have XYZ axis (in that order)
               
               Returns: str

        """
        if len(args) == 1:
            return self._findGeocentricCoordinateSystem_577fd326(*args)
        elif len(args) == 2:
            return self._findGeocentricCoordinateSystem_a0c8a943(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _findGeocentricCoordinateSystem_577fd326(self, csID):
        """Find a geocentric coordinate system that has the best match with the specified coordinate system.
           
           In most cases this returns the first geocentric coordinate system where no
           datum transformation is required. The returned string is the coordinate system's
           GeoSysManager UID. If no coordinate system could be found then an empty string
           is returned.
           
           New in version 1.3
           
           Arguments:
             csID (str): identifier of coordinate system
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = csID
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("findGeocentricCoordinateSystem_577fd326", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _findGeocentricCoordinateSystem_a0c8a943(self, csID, forceXYZ):
        """Find a geocentric coordinate system that has the best match with the specified coordinate system.
           
           In most cases this returns the first geocentric coordinate system where no
           datum transformation is required. The returned string is the coordinate system's
           GeoSysManager UID. If no coordinate system could be found then an empty string
           is returned.
           
           New in version 1.3
           
           Arguments:
             csID (str): identifier of coordinate system
             forceXYZ (bool): if true then the geocentric coordinate system must have XYZ axis (in that order)
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = csID
        inputs[1].i32 = 1 if forceXYZ else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("findGeocentricCoordinateSystem_a0c8a943", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
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

    def coordinateSystemsChanged(self):
        """This signal is emitted when the coordinate systems changed.
           
           Returns: ServiceSignal"""
        return self._coordinateSystemsChanged
