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


def _projectservice_error_decoder(payload):
    return json.loads(payload.decode())
def _projectservice_storagemediachanged_decoder(payload):
    return struct.unpack("!i", payload)[0]
def _projectservice_projectchanged_decoder(payload):
    return payload.decode()
def _projectservice_scanpositionchanged_decoder(payload):
    return payload.decode()

class ProjectService(object):
    """The project service.
       
       The project service provides functionality to manage projects and scan
       positions. It can be used to create and select a project or scan position
       and retrieve information about existing projects."""

    class StorageMedia(enum.IntEnum):
        SM_INTERNAL = 0
        SM_USB = 1
        SM_SDCARD = 2
        SM_NAS = 3

    class Error(enum.IntEnum):
        SUCCESS = 0
        UNIMPLEMENTED = 1
        PROJECT_DIR_NOT_FOUND = 2
        PROJECT_NOT_FOUND = 3
        PROJECT_EXISTS = 4
        PROJECT_CREATE_MKDIR_FAILED = 5
        INVALID_ARGUMENT = 10
        NO_PROJECT_LOADED = 11
        NO_SCANPOSITION_LOADED = 12
        SCANPOS_DIR_NOT_FOUND = 20
        SCANPOS_NOT_FOUND = 21
        SCANPOS_EXISTS = 22
        SCANPOS_CREATE_MKDIR_FAILED = 23
        INTERNAL_SERVER_ERROR = 255

    SM_INTERNAL = StorageMedia.SM_INTERNAL
    SM_USB = StorageMedia.SM_USB
    SM_SDCARD = StorageMedia.SM_SDCARD
    SM_NAS = StorageMedia.SM_NAS
    SUCCESS = Error.SUCCESS
    UNIMPLEMENTED = Error.UNIMPLEMENTED
    PROJECT_DIR_NOT_FOUND = Error.PROJECT_DIR_NOT_FOUND
    PROJECT_NOT_FOUND = Error.PROJECT_NOT_FOUND
    PROJECT_EXISTS = Error.PROJECT_EXISTS
    PROJECT_CREATE_MKDIR_FAILED = Error.PROJECT_CREATE_MKDIR_FAILED
    INVALID_ARGUMENT = Error.INVALID_ARGUMENT
    NO_PROJECT_LOADED = Error.NO_PROJECT_LOADED
    NO_SCANPOSITION_LOADED = Error.NO_SCANPOSITION_LOADED
    SCANPOS_DIR_NOT_FOUND = Error.SCANPOS_DIR_NOT_FOUND
    SCANPOS_NOT_FOUND = Error.SCANPOS_NOT_FOUND
    SCANPOS_EXISTS = Error.SCANPOS_EXISTS
    SCANPOS_CREATE_MKDIR_FAILED = Error.SCANPOS_CREATE_MKDIR_FAILED
    INTERNAL_SERVER_ERROR = Error.INTERNAL_SERVER_ERROR

    def __init__(self, address):
        self._svc = riconnect.Service("ProjectService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_projectservice_error_decoder)
        self._storageMediaChanged = ServiceSignal(self._svc, "storageMediaChanged", decoderFn=_projectservice_storagemediachanged_decoder)
        self._projectChanged = ServiceSignal(self._svc, "projectChanged", decoderFn=_projectservice_projectchanged_decoder)
        self._scanpositionChanged = ServiceSignal(self._svc, "scanpositionChanged", decoderFn=_projectservice_scanpositionchanged_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def setStorageMedia(self, value):
        """Set the active storage media.
           
           If the new storage media is different then the active storage media then
           the storageMediaChanged signal will be emitted. Changing the storage
           media might also cause the active project and scanposition to change.
           
           Arguments:
             value (StorageMedia): the new storage media"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(value)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setStorageMedia_b285ef5a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def storageMedia(self):
        """Return the active storage media.
           
           Returns: StorageMedia"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("storageMedia_3b72af71", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ProjectService.StorageMedia(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def createProject(self, *args):
        """
           createProject(name)
               Create a new project on the active storage media.
               
               Arguments:
                 name (str): the name of the project
               
               Returns: Error

           createProject(media, name)
               Create a new project on the specified storage media.
               
               New in version 1.2
               
               Arguments:
                 media (StorageMedia): the storage media
                 name (str): the name of the project
               
               Returns: Error

        """
        if len(args) == 1:
            return self._createProject_556861f3(*args)
        elif len(args) == 2:
            return self._createProject_0e87533a(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _createProject_556861f3(self, name):
        """Create a new project on the active storage media.
           
           Arguments:
             name (str): the name of the project
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("createProject_556861f3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ProjectService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _createProject_0e87533a(self, media, name):
        """Create a new project on the specified storage media.
           
           New in version 1.2
           
           Arguments:
             media (StorageMedia): the storage media
             name (str): the name of the project
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = int(media)
        inputs[1].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("createProject_0e87533a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ProjectService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def loadProject(self, name):
        """Load the specified project, making it active.
           
           Arguments:
             name (str): the name of the project
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("loadProject_288fb236", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ProjectService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def suggestProjectName(self):
        """Return a suggestion for a new project name.
           
           The created suggestion is based on the active project. The base name
           of the project is extracted and any available counter is incremented.
           e.g.
             activeProject    = Project001
             suggestedProject = Project002
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("suggestProjectName_208f3ca0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def logMessage(self, message):
        """Add message to project log.
           
           The project log is a text file called 'project.log', located in the
           project directory.
           
           Arguments:
             message (str): the log message"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = message
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("logMessage_5e703234", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def createScanposition(self, *args):
        """
           createScanposition(name)
               Create a new scanposition in the active project.
               
               Arguments:
                 name (str): scanposition name
               
               Returns: Error

           createScanposition(media, project, name)
               Create a new scanposition in the specified project.
               
               New in version 1.2
               
               Arguments:
                 media (StorageMedia): storage media
                 project (str): project name
                 name (str): scanposition name
               
               Returns: Error

        """
        if len(args) == 1:
            return self._createScanposition_0805a7bc(*args)
        elif len(args) == 3:
            return self._createScanposition_13b2e54c(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _createScanposition_0805a7bc(self, name):
        """Create a new scanposition in the active project.
           
           Arguments:
             name (str): scanposition name
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("createScanposition_0805a7bc", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ProjectService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _createScanposition_13b2e54c(self, media, project, name):
        """Create a new scanposition in the specified project.
           
           New in version 1.2
           
           Arguments:
             media (StorageMedia): storage media
             project (str): project name
             name (str): scanposition name
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 3)]
        inputs[0].i32 = int(media)
        inputs[1].s = project
        inputs[2].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("createScanposition_13b2e54c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ProjectService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def selectScanposition(self, name):
        """Select the specified scanposition, making it active.
           
           Arguments:
             name (str): the name of the scanposition
           
           Returns: Error"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("selectScanposition_9af004bd", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(ProjectService.Error(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def suggestScanpositionName(self):
        """Return a suggestion for a new scanposition name.
           
           The created suggestion is based on the active scanposition. The base name
           of the project is extracted and any available counter is incremented.
           e.g.
             activeScanposition    = ScanPos001
             suggestedScanposition = ScanPos002
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("suggestScanpositionName_3f25ccde", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def projectName(self):
        """Return the name of the active project.
           
           Returns an empty string if no project has been loaded.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("projectName_6be44560", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def scanpositionName(self):
        """Return the name of the active scanposition.
           
           Returns an empty string if no scanposition has been selected.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanpositionName_20e80977", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def projectPath(self, *args):
        """
           projectPath()
               The absolute path of the project directory.
               
               Returns an empty string if no project has been loaded.
               
               Returns: str

           projectPath(media, projectName)
               Return the absolute path of the specified project.
               
               Arguments:
                 media (StorageMedia): the storage media the project is stored on
                 projectName (str): the name of the project
               
               Returns: str

        """
        if len(args) == 0:
            return self._projectPath_171034f7(*args)
        elif len(args) == 2:
            return self._projectPath_451f6530(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _projectPath_171034f7(self):
        """The absolute path of the project directory.
           
           Returns an empty string if no project has been loaded.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("projectPath_171034f7", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _projectPath_451f6530(self, media, projectName):
        """Return the absolute path of the specified project.
           
           Arguments:
             media (StorageMedia): the storage media the project is stored on
             projectName (str): the name of the project
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = int(media)
        inputs[1].s = projectName
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("projectPath_451f6530", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def scanpositionPath(self, *args):
        """
           scanpositionPath()
               The path to the directory where scan position data should be stored.
               
               Returns an empty string if no scan position has been selected.
               
               Returns: str

           scanpositionPath(media, projectName, scanpositionName)
               Return the absolute path of the specified scan position.
               
               Arguments:
                 media (StorageMedia): the storage media the project is stored on
                 projectName (str): the name of the project
                 scanpositionName (str): the name of the scanposition
               
               Returns: str

        """
        if len(args) == 0:
            return self._scanpositionPath_a2e217aa(*args)
        elif len(args) == 3:
            return self._scanpositionPath_f2a1862a(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _scanpositionPath_a2e217aa(self):
        """The path to the directory where scan position data should be stored.
           
           Returns an empty string if no scan position has been selected.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanpositionPath_a2e217aa", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _scanpositionPath_f2a1862a(self, media, projectName, scanpositionName):
        """Return the absolute path of the specified scan position.
           
           Arguments:
             media (StorageMedia): the storage media the project is stored on
             projectName (str): the name of the project
             scanpositionName (str): the name of the scanposition
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 3)]
        inputs[0].i32 = int(media)
        inputs[1].s = projectName
        inputs[2].s = scanpositionName
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanpositionPath_f2a1862a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def projects(self, *args):
        """
           projects()
               Return the project information of the projects on the active storage media.
               
               The data is returned as JSON string in the following format:
               [
                 {
                   "name": "Project001",
                   "description": "",
                   "creationdate": "2015-10-31 09:39:00",
                   "location": {
                     "coordSystem": "",
                     "coord1": 0,
                     "coord2": 0,
                     "coord3": 0
                   }
                 },
                 ...
               ]
               
               Returns: str

           projects(media)
               Return the project information of the projects on the specified storage media.
               
               The data is returned as JSON string, see projects().
               
               Arguments:
                 media (StorageMedia): the storage media
               
               Returns: str

        """
        if len(args) == 0:
            return self._projects_35a2864c(*args)
        elif len(args) == 1:
            return self._projects_db92a2a5(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _projects_35a2864c(self):
        """Return the project information of the projects on the active storage media.
           
           The data is returned as JSON string in the following format:
           [
             {
               "name": "Project001",
               "description": "",
               "creationdate": "2015-10-31 09:39:00",
               "location": {
                 "coordSystem": "",
                 "coord1": 0,
                 "coord2": 0,
                 "coord3": 0
               }
             },
             ...
           ]
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("projects_35a2864c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _projects_db92a2a5(self, media):
        """Return the project information of the projects on the specified storage media.
           
           The data is returned as JSON string, see projects().
           
           Arguments:
             media (StorageMedia): the storage media
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(media)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("projects_db92a2a5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def scanpositions(self, *args):
        """
           scanpositions()
               Return the scanpositions of the active project.
               
               The data is returned as JSON string in the following format:
               [
                 {
                   "name": "ScanPos001"
                 },
                 ...
               ]
               
               Returns: str

           scanpositions(media, project)
               Return the scanpositions of the specified project.
               
               The data is returned as JSON string, see scanpositions().
               
               Arguments:
                 media (StorageMedia): the storage media
                 project (str): the name of the project
               
               Returns: str

        """
        if len(args) == 0:
            return self._scanpositions_cd12961e(*args)
        elif len(args) == 2:
            return self._scanpositions_e6ffb63a(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _scanpositions_cd12961e(self):
        """Return the scanpositions of the active project.
           
           The data is returned as JSON string in the following format:
           [
             {
               "name": "ScanPos001"
             },
             ...
           ]
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanpositions_cd12961e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _scanpositions_e6ffb63a(self, media, project):
        """Return the scanpositions of the specified project.
           
           The data is returned as JSON string, see scanpositions().
           
           Arguments:
             media (StorageMedia): the storage media
             project (str): the name of the project
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = int(media)
        inputs[1].s = project
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scanpositions_e6ffb63a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def scans(self, *args):
        """
           scans()
               Return information about the available scans of the active scanposition.
               
               The data is returned as JSON string in the following format:
               [
                 {
                   "name": "Scan001",
                  "path": "/media/intern/Project001.PROJ/ScanPos001.SCNPOS/scans",
                   "measurementFile": "160101_082200.rxp",
                   "monitorFile": "160101_082200.mon.rxp",
                   "previewFile": "160101_082200.png",
                   "metadataFile": "160101_082200.scn",
                   "fov": {...}
                 },
                 ...
               ]
               
               The attributes 'measurementFile' and 'monitorFile' are optional and are
               only present if the files do actually exist.
               
               Returns: str

           scans(media, project, scanposition)
               Return information about the available scans of the specified scanposition.
               
               The data is returned as JSON string, see scans().
               
               Arguments:
                 media (StorageMedia): the storage media
                 project (str): the name of the project
                 scanposition (str): the name of scan position
               
               Returns: str

        """
        if len(args) == 0:
            return self._scans_68bbe47a(*args)
        elif len(args) == 3:
            return self._scans_a763cc21(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _scans_68bbe47a(self):
        """Return information about the available scans of the active scanposition.
           
           The data is returned as JSON string in the following format:
           [
             {
               "name": "Scan001",
              "path": "/media/intern/Project001.PROJ/ScanPos001.SCNPOS/scans",
               "measurementFile": "160101_082200.rxp",
               "monitorFile": "160101_082200.mon.rxp",
               "previewFile": "160101_082200.png",
               "metadataFile": "160101_082200.scn",
               "fov": {...}
             },
             ...
           ]
           
           The attributes 'measurementFile' and 'monitorFile' are optional and are
           only present if the files do actually exist.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scans_68bbe47a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _scans_a763cc21(self, media, project, scanposition):
        """Return information about the available scans of the specified scanposition.
           
           The data is returned as JSON string, see scans().
           
           Arguments:
             media (StorageMedia): the storage media
             project (str): the name of the project
             scanposition (str): the name of scan position
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 3)]
        inputs[0].i32 = int(media)
        inputs[1].s = project
        inputs[2].s = scanposition
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("scans_a763cc21", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def images(self, *args):
        """
           images()
               Return information about the available images of the active scanposition.
               
               The data is returned as JSON string in the following format:
               [
                 {
                   "path": "/media/intern/Project001.PROJ/ScanPos001.SCNPOS",
                   "lineAngle": 0,
                   "frameAngle": 90,
                   "fileName": "160101_080120_2500ms_0.jpg",
                   "thumbnailName": "160101_080120_2500ms_0.thumb.jpg",
                   "metadataFile": "160101_080120_2500ms_0.img"
                 },
                 ...
               ]
               
               Returns: str

           images(media, project, scanposition)
               Return information about the available images of the specified scanposition.
               
               The data is returned as JSON string, see images().
               
               Arguments:
                 media (StorageMedia): the storage media containing the project
                 project (str): the name of the project
                 scanposition (str): the name of the scan position
               
               Returns: str

        """
        if len(args) == 0:
            return self._images_59b51417(*args)
        elif len(args) == 3:
            return self._images_bba4fd3d(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _images_59b51417(self):
        """Return information about the available images of the active scanposition.
           
           The data is returned as JSON string in the following format:
           [
             {
               "path": "/media/intern/Project001.PROJ/ScanPos001.SCNPOS",
               "lineAngle": 0,
               "frameAngle": 90,
               "fileName": "160101_080120_2500ms_0.jpg",
               "thumbnailName": "160101_080120_2500ms_0.thumb.jpg",
               "metadataFile": "160101_080120_2500ms_0.img"
             },
             ...
           ]
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("images_59b51417", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _images_bba4fd3d(self, media, project, scanposition):
        """Return information about the available images of the specified scanposition.
           
           The data is returned as JSON string, see images().
           
           Arguments:
             media (StorageMedia): the storage media containing the project
             project (str): the name of the project
             scanposition (str): the name of the scan position
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 3)]
        inputs[0].i32 = int(media)
        inputs[1].s = project
        inputs[2].s = scanposition
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("images_bba4fd3d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def tiepointscans(self, *args):
        """
           tiepointscans()
               Return information about the available tie point scans of the active scanposition.
               
               The data is returned as JSON string in the following format:
               [
                 {
                   "name": "160101_082500_TP0",
                  "path": "/media/intern/Project001.PROJ/ScanPos001.SCNPOS/tiepointscans",
                   "measurementFile": "160101_082500_TP0.rxp",
                   "previewFile": "160101_082500_TP0.png",
                   "metadataFile": "160101_082500_TP0.tp",
                   "fov": {...}
                 },
                 ...
               ]
               
               Returns: str

           tiepointscans(media, project, scanposition)
               Return information about the available tie point scans of the specified scanposition.
               
               The data is returned as JSON string, see tiepointscans().
               
               Arguments:
                 media (StorageMedia): the storage media containing the project
                 project (str): the name of the project
                 scanposition (str): the name of the scan position
               
               Returns: str

        """
        if len(args) == 0:
            return self._tiepointscans_71806058(*args)
        elif len(args) == 3:
            return self._tiepointscans_1b3ab54c(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _tiepointscans_71806058(self):
        """Return information about the available tie point scans of the active scanposition.
           
           The data is returned as JSON string in the following format:
           [
             {
               "name": "160101_082500_TP0",
              "path": "/media/intern/Project001.PROJ/ScanPos001.SCNPOS/tiepointscans",
               "measurementFile": "160101_082500_TP0.rxp",
               "previewFile": "160101_082500_TP0.png",
               "metadataFile": "160101_082500_TP0.tp",
               "fov": {...}
             },
             ...
           ]
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("tiepointscans_71806058", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _tiepointscans_1b3ab54c(self, media, project, scanposition):
        """Return information about the available tie point scans of the specified scanposition.
           
           The data is returned as JSON string, see tiepointscans().
           
           Arguments:
             media (StorageMedia): the storage media containing the project
             project (str): the name of the project
             scanposition (str): the name of the scan position
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 3)]
        inputs[0].i32 = int(media)
        inputs[1].s = project
        inputs[2].s = scanposition
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("tiepointscans_1b3ab54c", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setProjectLocation(self, coordSystem, coord1, coord2, coord3):
        """Set the current project's location information.
           
           Arguments:
             coordSystem (str): the EPSG code (e.g. 'EPSG::4978') or name of the coordinate system
             coord1 (float): value of first coordinate axis
             coord2 (float): value of second coordinate axis
             coord3 (float): value of third coordinate axis"""
        inputs = [riconnect.Value() for i in range(0, 4)]
        inputs[0].s = coordSystem
        inputs[1].d = coord1
        inputs[2].d = coord2
        inputs[3].d = coord3
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setProjectLocation_383fa642", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def project(self):
        """Return information about the active project.
           
           The data is returned as JSON string in the following format:
           {
             "name": "Project001",
             "path": "/media/intern/projects/Project001.PROJ",
             "location": { "coordSystem": "", "coord1": 0, "coord2": 0, "coord3": 0}
           }
           
           The project's location has not been set if either the 'location'
           attribute does not exist or the 'coordSystem' is empty.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("project_46f86faa", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
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

    def storageMediaChanged(self):
        """This signal is emitted when the active storage media changed.
           
           The active storage media can change due to a call of setStorageMedia()
           or if the media was ejected (e.g. USB, or SD-Card).
           
           Returns: ServiceSignal
           Payload: StorageMedia"""
        return self._storageMediaChanged

    def projectChanged(self):
        """This signal is emitted when the active project changed.
           
           The signal payload is the name of the new active project.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._projectChanged

    def scanpositionChanged(self):
        """This signal is emitted when the active scanposition changed.
           
           The signal payload is the name of the new active scanposition.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._scanpositionChanged
