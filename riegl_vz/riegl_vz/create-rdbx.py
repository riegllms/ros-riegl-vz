#!/bin/env python3

import os
from os.path import join
import sys
import argparse
import json
import signal
import time
from threading import Event
from vzi_services.projectservice import ProjectService
from vzi_services.controlservice import ControlService
from vzi_services.dataprocservice import DataprocService

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
    scan: str):
    """Create rdbx from rxp. Blocks until conversion is finished."""
    taskId = None

    taskFinishedEvent = Event()
    def onBackgroundTaskRemoved(arg0):
        obj = json.loads(arg0)
        if obj.get('id') == taskId:
            taskFinishedEvent.set()

    sigcon = ctrlSvc.backgroundTaskRemoved().connect(onBackgroundTaskRemoved)

    taskId = ctrlSvc.addRdbCreationTask(
        storMedia,
        projectName,
        scanposName,
        scan
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

def createArgumentParser():
    parser = argparse.ArgumentParser(description="Perform data acquisition.")
    parser.add_argument('--connectionstring',
        default='127.0.0.1:20000',
        help='address of scanner services')
    parser.add_argument('--project',
        help='project name')
    parser.add_argument('--scanposition',
        help='scanposition name')
    parser.add_argument('--scan',
        help='scan name')
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
    procSvc = DataprocService(args.connectionstring)
    ctrlSvc = ControlService(args.connectionstring)

    scan = ''
    if args.scan:
        scan = args.scan
    else:
        scanId = procSvc.actualFile(0)
        scan: str = os.path.basename(scanId).replace(".rxp", "")[0:13] 
    storMedia = mediaString(projSvc)
    if not createRdbx(
        sigHandler, ctrlSvc,
        storMedia,
        args.project,
        args.scanposition,
        scan):
        print("RDB creation canceled.")
        return

if __name__ == "__main__":
    main()
