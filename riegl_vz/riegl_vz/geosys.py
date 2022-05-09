import json
import threading

from rclpy.node import Node

from vzi_services.geosysservice import GeoSysService

class RieglVzGeoSys():
    def __init__(self, node):
        self._node = node
        self._hostname = node.hostname
        self._connectionString = self._hostname + ':20000'
        self._logger = node.get_logger()
        self._csDict = {}
        self._geosysSvc = None
        self._threadLock = threading.Lock()

    def _lock(self):
        self._threadLock.acquire()

    def _unlock(self):
        self._threadLock.release()

    def _connect(self):
        if not self._geosysSvc:
            try:
                self._geosysSvc = GeoSysService(self._connectionString)
                self._geosysSvc.reloadCoordinateSystems()
            except:
                self._geosysSvc = None
                self._logger.error("GeoSysService is not available!")
                return False
        return True

    def transformToWgs84(self, cs, coord1, coord2, coord3):
        ok = False
        dstCoord1 = 0
        dstCoord2 = 0
        dstCoord3 = 0
        self._lock()
        if self._connect():
            try:
                if self._csDict.get(cs) is None:
                    self._csDict[cs] = json.loads(self._geosysSvc.findCoordinateSystem(cs))
                    dstCoords = json.loads(self._geosysSvc.transformToWgs84(self._csDict[cs]['uid'], coord1, coord2, coord3))
                ok = True
                dstCoord1 = dstCoords["coord1"]
                dstCoord2 = dstCoords['coord2']
                dstCoord3 = dstCoords['coord3']
            except:
                pass
        self.unlock()
        return ok, float(coord1), float(coord2), float(coord3)


    def transformCoordinate(self, srcCs, dstCs, coord1, coord2, coord3):
        ok = False
        dstCoord1 = 0
        dstCoord2 = 0
        dstCoord3 = 0
        self._lock()
        if self._connect():
            try:
                if self._csDict.get(srcCs) is None:
                    self._csDict[srcCs] = json.loads(self._geosysSvc.findCoordinateSystem(srcCs))
                if self._csDict.get(dstCs) is None:
                    self._csDict[dstCs] = json.loads(self._geosysSvc.findCoordinateSystem(dstCs))
                dstCoords = json.loads(self._geosysSvc.transformCoordinate(self._csDict[srcCs]['uid'], self._csDict[dstCs]['uid'], coord1, coord2, coord3))
                ok = True
                dstCoord1 = dstCoords["coord1"]
                dstCoord2 = dstCoords['coord2']
                dstCoord3 = dstCoords['coord3']
            except:
                pass
        self._unlock()
        return ok, float(dstCoord1), float(dstCoord2), float(dstCoord3)
