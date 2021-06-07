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


class ActiveStorageProviderChangedPayload(dict):
    def __init__(self, *args, **kwargs):
        self['oldValue'] = None
        self['newValue'] = None
        super().__init__(*args, **kwargs)
    @property
    def oldValue(self):
        return self['oldValue']
    @oldValue.setter
    def oldValue(self, value):
        self['oldValue'] = value
    @property
    def newValue(self):
        return self['newValue']
    @newValue.setter
    def newValue(self, value):
        self['newValue'] = value

class UploadTargets(dict):
    def __init__(self, *args, **kwargs):
        self['localFilenames'] = []
        self['remoteFilenames'] = []
        super().__init__(*args, **kwargs)
    @property
    def localFilenames(self):
        return self['localFilenames']
    @localFilenames.setter
    def localFilenames(self, value):
        self['localFilenames'] = value
    @property
    def remoteFilenames(self):
        return self['remoteFilenames']
    @remoteFilenames.setter
    def remoteFilenames(self, value):
        self['remoteFilenames'] = value

def _cloudstorageservice_error_decoder(payload):
    return json.loads(payload.decode())
def _cloudstorageservice_activestorageproviderchanged_decoder(payload):
    return ActiveStorageProviderChangedPayload(json.loads(payload.decode()))
def _cloudstorageservice_fileuploadstarted_decoder(payload):
    return payload.decode()
def _cloudstorageservice_fileuploadprogress_decoder(payload):
    return payload.decode()
def _cloudstorageservice_fileuploadfinished_decoder(payload):
    return payload.decode()
def _cloudstorageservice_pendingfileuploadschanged_decoder(payload):
    return payload.decode()
def _cloudstorageservice_storageproviderconfigurationchanged_decoder(payload):
    return payload.decode()
def _cloudstorageservice_filedownloadstarted_decoder(payload):
    return payload.decode()
def _cloudstorageservice_filedownloadprogress_decoder(payload):
    return payload.decode()
def _cloudstorageservice_filedownloadfinished_decoder(payload):
    return payload.decode()
def _cloudstorageservice_listfilesasyncresponse_decoder(payload):
    return payload.decode()

class CloudStorageService(object):
    """Brief service description.
       
       Detailed service description."""

    class StorageProvider(enum.IntEnum):
        CSP_FTP = 1
        CSP_AMAZON_S3 = 2
        CSP_MICROSOFT_AZURE = 3

    CSP_FTP = StorageProvider.CSP_FTP
    CSP_AMAZON_S3 = StorageProvider.CSP_AMAZON_S3
    CSP_MICROSOFT_AZURE = StorageProvider.CSP_MICROSOFT_AZURE

    def __init__(self, address):
        self._svc = riconnect.Service("CloudStorageService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_cloudstorageservice_error_decoder)
        self._activeStorageProviderChanged = ServiceSignal(self._svc, "activeStorageProviderChanged", decoderFn=_cloudstorageservice_activestorageproviderchanged_decoder)
        self._fileUploadStarted = ServiceSignal(self._svc, "fileUploadStarted", decoderFn=_cloudstorageservice_fileuploadstarted_decoder)
        self._fileUploadProgress = ServiceSignal(self._svc, "fileUploadProgress", decoderFn=_cloudstorageservice_fileuploadprogress_decoder)
        self._fileUploadFinished = ServiceSignal(self._svc, "fileUploadFinished", decoderFn=_cloudstorageservice_fileuploadfinished_decoder)
        self._pendingFileUploadsChanged = ServiceSignal(self._svc, "pendingFileUploadsChanged", decoderFn=_cloudstorageservice_pendingfileuploadschanged_decoder)
        self._storageProviderConfigurationChanged = ServiceSignal(self._svc, "storageProviderConfigurationChanged", decoderFn=_cloudstorageservice_storageproviderconfigurationchanged_decoder)
        self._fileDownloadStarted = ServiceSignal(self._svc, "fileDownloadStarted", decoderFn=_cloudstorageservice_filedownloadstarted_decoder)
        self._fileDownloadProgress = ServiceSignal(self._svc, "fileDownloadProgress", decoderFn=_cloudstorageservice_filedownloadprogress_decoder)
        self._fileDownloadFinished = ServiceSignal(self._svc, "fileDownloadFinished", decoderFn=_cloudstorageservice_filedownloadfinished_decoder)
        self._listFilesAsyncResponse = ServiceSignal(self._svc, "listFilesAsyncResponse", decoderFn=_cloudstorageservice_listfilesasyncresponse_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def listTargets(self, *args):
        """
           listTargets(sp)
               Return a list of available storage targets for the specified storage provider.
               
               The result is a JSON string containing at least the following information.
               [
                 { "name": "the-name" },
                ...
               ]
               
               For an Amazon S3 storage the returned entries represent the storage buckets.
               For an FTP storage an empty list is returned.
               
               Arguments:
                 sp (StorageProvider): The storage provider.
               
               Returns: str

           listTargets()
               This is an overloaded function.
               
               Returns a list of available storage targets for the active storage provider.
               
               Returns: str

        """
        if len(args) == 1:
            return self._listTargets_dd6c7650(*args)
        elif len(args) == 0:
            return self._listTargets_129328d4(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _listTargets_dd6c7650(self, sp):
        """Return a list of available storage targets for the specified storage provider.
           
           The result is a JSON string containing at least the following information.
           [
             { "name": "the-name" },
            ...
           ]
           
           For an Amazon S3 storage the returned entries represent the storage buckets.
           For an FTP storage an empty list is returned.
           
           Arguments:
             sp (StorageProvider): The storage provider.
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(sp)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("listTargets_dd6c7650", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=120.0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _listTargets_129328d4(self):
        """This is an overloaded function.
           
           Returns a list of available storage targets for the active storage provider.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("listTargets_129328d4", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=120.0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def listFiles(self, *args):
        """
           listFiles(sp, path)
               Return a list of files in the specified cloud storage.
               
               The result is a JSON string containing at least the following information.
               [
                 { "name": "itemName" },
                ...
               ]
               
               Arguments:
                 sp (StorageProvider): Storage provider.
                 path (str): The directory path (or storage item prefix).
               
               Returns: str

           listFiles(path)
               This is an overloaded function.
               
               Returns a list of files in the active cloud storage.
               
               Arguments:
                 path (str): 
               
               Returns: str

        """
        if len(args) == 2:
            return self._listFiles_17edb3ff(*args)
        elif len(args) == 1:
            return self._listFiles_1d071a9e(*args)
        else:
            raise RuntimeError("Invalid function arguments.")

    def _listFiles_17edb3ff(self, sp, path):
        """Return a list of files in the specified cloud storage.
           
           The result is a JSON string containing at least the following information.
           [
             { "name": "itemName" },
            ...
           ]
           
           Arguments:
             sp (StorageProvider): Storage provider.
             path (str): The directory path (or storage item prefix).
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = int(sp)
        inputs[1].s = path
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("listFiles_17edb3ff", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=120.0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def _listFiles_1d071a9e(self, path):
        """This is an overloaded function.
           
           Returns a list of files in the active cloud storage.
           
           Arguments:
             path (str): 
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = path
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("listFiles_1d071a9e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=120.0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def listFilesAsync(self, requestId, path):
        """Return a list of files in the active cloud storage.
           
           This method call is asynchronous. The result will be emitted through
           the listFilesAsyncResponse signal.
           
           New in version 1.3.
           
           Arguments:
             requestId (str): ID of the asynchronous request.
             path (str): The directory path (or storage item prefix)."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = requestId
        inputs[1].s = path
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("listFilesAsync_c81ea93f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def activeStorageProvider(self):
        """Return the active cloud storage provider.
           
           Returns: StorageProvider"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("activeStorageProvider_14e42a9e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(CloudStorageService.StorageProvider(rvalues[0].i32))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setActiveStorageProvider(self, sp):
        """Set the active cloud storage provider.
           
           Arguments:
             sp (StorageProvider): The new storage provider."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(sp)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setActiveStorageProvider_5cc00dff", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def storageProviderConfiguration(self, sp):
        """Return the configuration of the specified cloud storage provider.
           
           The returned string represents a JSON object with the configuration information
           (like username, authentication key, etc.) of the storage provider.
           
           Arguments:
             sp (StorageProvider): 
           
           Returns: str"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(sp)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("storageProviderConfiguration_78a9755b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setStorageProviderConfiguration(self, sp, jsonString):
        """Set the configuration information of a cloud storage provider.
           
           FTP:
           { "base_path": "files/abc" }
           
           Microsoft Azure
           { "container": "mycontainer" }
           
           Amazon S3
           { "region": "eu-central-1", "bucket": "mybucket" }
           
           Arguments:
             sp (StorageProvider): The storage provider.
             jsonString (str): Configuration as JSON object."""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = int(sp)
        inputs[1].s = jsonString
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setStorageProviderConfiguration_665629dc", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def queueFileUpload(self, source, destination):
        """Queue a file for upload to cloud storage.
           
           If the specified source is a directory then all files in the directory
           and sub-directories are queued for upload.
           
           Returns the ID of the created file upload or 0 if the file does not exist.
           If source is a directory then the return value represents the number of
           files that were queued for upload.
           
           Queued file uploads are processed sequentially. Only one file is uploaded
           at a time.
           
           Arguments:
             source (str): Name of the local file.
             destination (str): Name of the remote file.
           
           Returns: int"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = source
        inputs[1].s = destination
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("queueFileUpload_d78cfde5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def queueFileUploads(self, tagets):
        """Queue files for upload to cloud storage.
           
           The return value represents the number of files that were queued for
           upload. If the number of remote file names does not match the number of
           local file names then no file is queued for upload.
           
           Queued file uploads are processed sequentially. Only one file is uploaded
           at a time.
           
           Arguments:
             tagets (UploadTargets): Files to upload.
           
           Returns: int"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = json.dumps(tagets)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("queueFileUploads_7ef1e56e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def isUploading(self):
        """Return true if a file upload is pending (in progress, or queued).
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isUploading_489e66f6", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def cancelFileUpload(self, id):
        """Cancel the file upload with the specified ID.
           
           Arguments:
             id (int): File upload ID."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("cancelFileUpload_dd0c7f31", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def cancelFileUploadByName(self, source):
        """Cancel all uploads of the specified file.
           
           Arguments:
             source (str): Name of the local file."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = source
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("cancelFileUploadByName_7f142e7d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def cancelAllFileUploads(self):
        """Cancel all pending file uploads."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("cancelAllFileUploads_1c426ebc", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def cancelQueuedFileUploads(self):
        """Cancel all queued (not active) file uploads.
           
           New in version 1.2."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("cancelQueuedFileUploads_1e6b8d3d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def pendingFileUploads(self):
        """Return all pending file uploads.
           
           The return value is a JSON array in the following format:
           [
             { "id": 1, "filename": "/path/to/local/file1" },
             { "id": 2, "filename": "/path/to/local/file2" },
             ...
           ]
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("pendingFileUploads_91aed7f0", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def downloadFile(self, source, destination):
        """Download file from active cloud storage provider to local storage.
           
           Returns the ID of the created file download.
           
           New in version 1.2.
           
           Arguments:
             source (str): Name of the remote file.
             destination (str): Name of local file.
           
           Returns: int"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = source
        inputs[1].s = destination
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("downloadFile_49120047", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def cancelFileDownload(self, id):
        """Cancel the file download with the specified ID.
           
           New in version 1.2.
           
           Arguments:
             id (int): File download ID."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = id
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("cancelFileDownload_f7bff2e2", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def pendingFileDownloads(self):
        """Return all pending file downloads.
           
           The return value is a JSON array in the following format:
           [
             { "id": 1, "filename": "/path/to/local/file1" },
             { "id": 2, "filename": "/path/to/local/file2" },
             ...
           ]
           
           New in version 1.2.
           
           Returns: str"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("pendingFileDownloads_75b3a7fc", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].s)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def uploadFile(self, source, destination):
        """Upload file to cloud storage.
           
           Returns the ID of the created file upload.
           
           New in version 1.2.
           
           Arguments:
             source (str): Name of the local file.
             destination (str): Name of the remote file.
           
           Returns: int"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = source
        inputs[1].s = destination
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("uploadFile_c1fdd544", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def removeFile(self, path):
        """Remove file from active cloud storage provider.
           
           New in version 1.2.
           
           Arguments:
             path (str): The remote file identifier."""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = path
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("removeFile_a684b63f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
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

    def activeStorageProviderChanged(self):
        """This signal is emitted when the active storage provider changed.
           
           Returns: ServiceSignal
           Payload: ActiveStorageProviderChangedPayload"""
        return self._activeStorageProviderChanged

    def fileUploadStarted(self):
        """This signal is emitted when a file upload started.
           
           The signal payload is a JSON object with the following format:
           {
             "id": 1,
             "filename": "/path/to/local/file"
           }
           
           Returns: ServiceSignal
           Payload: str"""
        return self._fileUploadStarted

    def fileUploadProgress(self):
        """This signal is emitted when the file upload progress changed.
           
           The signal payload is a JSON object with the following format:
           {
             "id": 1,
             "filename": "/path/to/local/file",
             "progress": 30.5,
             "bytes_uploaded": 481576,
             "bytes_total": 1578940
           }
           
           Returns: ServiceSignal
           Payload: str"""
        return self._fileUploadProgress

    def fileUploadFinished(self):
        """This signal is emitted when a file upload finished.
           
           The signal payload is a JSON object with the following format:
           {
             "id": 1,
             "filename": "/path/to/local/file",
             "success": true,
             "aborted": false,
             "errorstring": ""
           }
           
           Returns: ServiceSignal
           Payload: str"""
        return self._fileUploadFinished

    def pendingFileUploadsChanged(self):
        """This signal is emitted when the pending file uploads changed.
           
           The pending file uploads do change when file uploads are queued, canceled
           or have finished (with success or failure).
           
           The signal payload is a JSON object with the following format:
           {
             "added":   [ { "id": 5, "filename": "path" }, ...],
             "removed": [ { "id": 2, "filename": "path" }, ...]
           }
           
           Returns: ServiceSignal
           Payload: str"""
        return self._pendingFileUploadsChanged

    def storageProviderConfigurationChanged(self):
        """This signal is emitted when the configuration of a storage provider changed.
           
           The signal payload is a JSON object with the following format:
           {
             "storageprovider": "ftp",
             "base_path": "files/abc"
           }
           
           The storage provider attribute is a string that can be one of "ftp",
           "s3", or "azure". The other attributes of the result depend on the
           storage provider. See setStorageProviderConfiguration().
           
           Returns: ServiceSignal
           Payload: str"""
        return self._storageProviderConfigurationChanged

    def fileDownloadStarted(self):
        """This signal is emitted when a file download started.
           
           The signal payload is a JSON object with the following format:
           {
             "id": 1,
             "filename": "/path/to/local/file"
           }
           
           New in version 1.2.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._fileDownloadStarted

    def fileDownloadProgress(self):
        """This signal is emitted when the file download progress changed.
           
           The signal payload is a JSON object with the following format:
           {
             "id": 1,
             "filename": "/path/to/local/file",
             "progress": 30.5,
             "bytes_downloaded": 481576,
             "bytes_total": 1578940
           }
           
           New in version 1.2.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._fileDownloadProgress

    def fileDownloadFinished(self):
        """This signal is emitted when a file download finished.
           
           The signal payload is a JSON object with the following format:
           {
             "id": 1,
             "filename": "/path/to/local/file",
             "success": true,
             "aborted": false,
             "errorstring": ""
           }
           
           New in version 1.2.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._fileDownloadFinished

    def listFilesAsyncResponse(self):
        """This signal is the result of the listFilesAsync method.
           
           The signal payload is a JSON object with the following format:
           {
             "request_id": "my_request_id",
             "success": true,
             "errorstring": "",
             "files": [
               { "name": "itemName" },
                ...
             ]
           }
           
           New in version 1.3.
           
           Returns: ServiceSignal
           Payload: str"""
        return self._listFilesAsyncResponse
