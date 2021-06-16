#!/bin/env python3

import os
from os.path import join
import sys
import argparse
import json
import signal
import time
import subprocess
from threading import Event
from vzi_services.projectservice import ProjectService
from vzi_services.scannerservice import ScannerService, RectScanPattern
from vzi_services.controlservice import ControlService, ReflectorSearchSettings, ReflectorScanSettings
from vzi_services.dataprocservice import DataprocService

appBinDir = appDir = os.path.dirname(os.path.abspath(__file__))

class SignalHandler(object):
    def __init__(self):
        self.canceled = False
        signal.signal(signal.SIGINT, self._handleSignal)
        signal.signal(signal.SIGTERM, self._handleSignal)

    def _handleSignal(self, signal_number, frame):
        self.canceled = True

def mediaString(projSvc):
    """Return string representation of active storage media (used by ControlService)."""
    media = projSvc.storageMedia()
    if media == ProjectService.SM_USB:
        return "USB"
    if media == ProjectService.SM_SDCARD:
        return "SDCARD"
    if media == ProjectService.SM_NAS:
        return "NAS"
    return "SSD"

def createRdbx(sigHandler, ctrlSvc,
    storMedia: str,
    projectName: str,
    scanposName: str,
    scanId: str):
    """Create rdbx from rxp. Blocks until conversion is finished."""
    taskId = None

    taskFinishedEvent = Event()
    def onBackgroundTaskRemoved(arg0):
        obj = json.loads(arg0)
        if obj.get('id') == taskId:
            taskFinishedEvent.set()

    sigcon = ctrlSvc.backgroundTaskRemoved().connect(onBackgroundTaskRemoved)

    if createRdbx:
        taskId = ctrlSvc.addRdbCreationTask(
            storMedia,
            projectName,
            scanposName,
            scanId
        )

    while not sigHandler.canceled:
        if taskFinishedEvent.is_set():
            break
        time.sleep(0.2)
    if sigHandler.canceled:
        try:
            if taskId:
                ctrlSvc.cancelBackgroundTask(taskId)
        except Exception:
            pass
        return False

    sigcon.disconnect()

    return True

def acquireData(
    sigHandler, scanSvc, ctrlSvc, procSvc,
    scanPattern: RectScanPattern,
    measProg: int,
    createRdbx=True,
    reflSearch: ReflectorSearchSettings = None,
    lineStep: int = 1,
    echoStep: int = 1,
    captureImage: bool = False,
    captureMode: int = 1,
    imageOverlap: int = 25
):
    """Perform data acquisition. Blocks until acquisition is finished."""
    scanSvc.setRectScan(scanPattern, 1)

    if scanSvc.activeMeasurementProgram() != measProg:
        scanSvc.setMeasurementProgram(measProg)

    finishedEvent = Event()
    def onDataAcquisitionFinished(arg0):
        finishedEvent.set()

    dataprocStarted = Event()
    def onDataProcessingStarted():
        dataprocStarted.set()

    flags = int(ControlService.ACQ_SCAN)
    # the IMU delay is needed for the relative pose estimation used during registration
    flags = flags | ControlService.ACQ_FINALIZATION_IMU_DELAY
    if reflSearch:
        ctrlSvc.setReflectorSearchSettings(reflSearch)
        refScan = ReflectorScanSettings()
        refScan.overlap = 10.0
        refScan.oversize = 2.0
        ctrlSvc.setReflectorScanSettings(refScan)
        flags = flags | ControlService.ACQ_REFLSEARCH | ControlService.ACQ_REFLSCANS

    if captureImage:
        if captureMode == 1:
            flags = flags | ControlService.ACQ_IMAGES_DURING_SCAN
        else:
            flags = flags | ControlService.ACQ_IMAGES_AFTER_SCAN
        ctrlSvc.setHorizontalImageOverlap(imageOverlap)
        ctrlSvc.setImageCaptureMode(ControlService.ICM_AUTOMATIC)

    # start data acquisition
    sigcon = ctrlSvc.acquisitionFinished().connect(onDataAcquisitionFinished)
    sigcon2 = procSvc.started().connect(onDataProcessingStarted)

    ctrlSvc.setStoreMeasurementStream(True)
    ctrlSvc.setStoreMonitorStream(True)
    ctrlSvc.startAcquisition(flags)

    # wait until acquisition is done
    while not sigHandler.canceled:
        if finishedEvent.is_set():
            break
        time.sleep(0.2)
    sigcon.disconnect()
    sigcon2.disconnect()

    acqInfo = ctrlSvc.lastAcquisition()
    if not acqInfo.success:
        raise RuntimeError("Data acquisition failed. {1}".format(acqInfo.errorMessage))

    return False if acqInfo.canceled else True

def extractReflectorSearchSettings(filepath):
    rss = None
    with open(filepath, "r") as f:
        obj = json.load(f)
        searchMode = obj.get("searchMode")
        if searchMode == "model":
            rss = ReflectorSearchSettings()
            rss.mode = searchMode
            for model in obj.get("searchModels"):
                modelName = model.get("name")
                if modelName:
                    rss.models.append(modelName)
        elif searchMode == "simple":
            rss = ReflectorSearchSettings()
            rss.mode = searchMode
            rss.minReflectance = obj["searchReflectance"]
            rss.minDiameter = obj["searchMinDiameter"]
            rss.maxDiameter = obj["searchMaxDiameter"]
        if rss:
            # shared configuration
            rss.resolveMta = obj.get("searchResolveMta", False)
            if obj.get("searchMinRange") is not None:
                rss.minRange = obj.get("searchMinRange")
            if obj.get("searchMaxRange") is not None:
                rss.maxRange = obj.get("searchMaxRange")
    return rss

def createArgumentParser():
    parser = argparse.ArgumentParser(description="Perform data acquisition.")
    parser.add_argument('--connectionstring',
        default='127.0.0.1:20000',
        help='address of scanner services')
    parser.add_argument('--project',
        help='project name')
    parser.add_argument('--scanposition',
        help='scanposition name')
    parser.add_argument('--reflsearch',
        help='file path of JSON file containing reflector search settings')
    parser.add_argument('--create-rdbx', action="store_true",
        help='enable creation of RDBX')
    parser.add_argument('--line-step', type=int,
        help='line step size for scan data acquisition')
    parser.add_argument('--echo-step', type=int,
        help='echo step size for scan data acquisition')
    parser.add_argument('--line-start', type=float, default=30.0,
        help='line start angle in degrees (default=30.0)')
    parser.add_argument('--line-stop', type=float, default=130.0,
        help='line stop angle in degrees (default=130.0)')
    parser.add_argument('--line-incr', type=float, default=0.04,
        help='line angle increment in degrees (default=0.04)')
    parser.add_argument('--frame-start', type=float, default=0.0,
        help='frame start angle in degrees (default=0.0)')
    parser.add_argument('--frame-stop', type=float, default=360.0,
        help='frame stop angle in degrees (default=360.0)')
    parser.add_argument('--frame-incr', type=float, default=0.04,
        help='frame angle increment in degrees (default=0.04)')
    parser.add_argument('--measprog', type=int, default=3,
        help='measurement program (default=3)')
    parser.add_argument('--capture-images', action="store_true",
        help='enable image acquisition (during scan)')
    parser.add_argument('--capture-mode', type=int, default=1,
        help='image capture mode. 1=during-scan, 2=after-scan (default=1)')
    parser.add_argument('--image-overlap', type=int, default=25,
        help='image acquisition overlap in percent (default=25)')
    return parser

def main():
    parser = createArgumentParser()
    args = parser.parse_args()

    # verify command line options
    if not args.project:
        print("No project name specified.")
        sys.exit(1)
    if not args.scanposition:
        print("No scanposition name specified.")
        sys.exit(2)

    sigHandler = SignalHandler()
    projSvc = ProjectService(args.connectionstring)
    scanSvc = ScannerService(args.connectionstring)
    procSvc = DataprocService(args.connectionstring)
    ctrlSvc = ControlService(args.connectionstring)

    # prepare project
    projSvc.createProject(args.project)
    projSvc.loadProject(args.project)
    projSvc.createScanposition(args.scanposition)
    projSvc.selectScanposition(args.scanposition)
    reflSearch = extractReflectorSearchSettings(args.reflsearch) if args.reflsearch else None

    scanPattern = RectScanPattern()
    scanPattern.thetaStart = args.line_start
    scanPattern.thetaStop = args.line_stop
    scanPattern.thetaIncrement = args.line_incr
    scanPattern.phiStart = args.frame_start
    scanPattern.phiStop = args.frame_stop
    scanPattern.phiIncrement = args.frame_incr
    measProg = args.measprog

    acquired = acquireData(
        sigHandler, scanSvc, ctrlSvc, procSvc,
        scanPattern, measProg,
        createRdbx=args.create_rdbx,
        reflSearch=reflSearch,
        lineStep=args.line_step,
        echoStep=args.echo_step,
        captureImage=args.capture_images,
        captureMode=args.capture_mode,
        imageOverlap=args.image_overlap)
    if not acquired:
        print("Data acquisition canceled.")
        return

    if createRdbx:
        scanId = procSvc.actualFile(0)
        storMedia = mediaString(projSvc)
        if not createRdbx(
            sigHandler, ctrlSvc,
            storMedia,
            args.project,
            args.scanposition,
            scanId):
            print("RDB creation canceled.")
            return


if __name__ == "__main__":
    main()
