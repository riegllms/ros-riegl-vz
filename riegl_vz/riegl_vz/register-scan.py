#!/bin/env python3

import sys
import argparse
import json
import signal
import time
from threading import Event
from vzi_services.projectservice import ProjectService
from vzi_services.controlservice import ControlService

class SignalHandler(object):
    def __init__(self):
        self.canceled = False
        signal.signal(signal.SIGINT, self._handleSignal)
        signal.signal(signal.SIGTERM, self._handleSignal)

    def _handleSignal(self, signal_number, frame):
        self.canceled = True

def registerScanposition(sigHandler, ctrlSvc, media, project, scanposition, mode):
    """Register scanposition. Blocks until registration is finished."""
    taskId = None
    finishedEvent = Event()
    def onBackgroundTaskRemoved(arg0):
        obj = json.loads(arg0)
        if obj.get('id') == taskId:
            finishedEvent.set()

    sigcon = ctrlSvc.backgroundTaskRemoved().connect(onBackgroundTaskRemoved)
    taskId = ctrlSvc.addRegistrationTask(media, project, scanposition, mode)
    while not sigHandler.canceled:
        if finishedEvent.is_set():
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

def mediaString(projSvc):
    """Return string representation of active storage media (used by ControlService)."""
    media = projSvc.storageMedia()
    if media == ProjectService.SM_USB:
        return 'USB'
    if media == ProjectService.SM_SDCARD:
        return 'SDCARD'
    if media == ProjectService.SM_NAS:
        return 'NAS'
    return 'SSD'

def main():
    parser = argparse.ArgumentParser(description='Register scan position.')
    parser.add_argument('--connectionstring',
        default='127.0.0.1:20000',
        help='address of scanner services')
    parser.add_argument('--project',
        help='project name')
    parser.add_argument('--scanposition',
        help='scanposition name')
    parser.add_argument('--registrationmode', type=int, default=1,
        help='the registration mode')
    args = parser.parse_args()

    # verify command line options
    if not args.project:
        print("No project name specified.")
        sys.exit(1)
    if not args.scanposition:
        print("No scanposition name specified.")
        sys.exit(2)
    if args.registrationmode < 1 or args.registrationmode > 7:
        print("""\
Invalid registration mode.
Supported values:
    1 ... AUTO
    2 ... OUTDOOR_URBAN
    3 ... OUTDOOR_NON_URBAN
    4 ... INDOOR_SMALL
    5 ... INDOOR_LARGE
    6 ... MINING_MEDIUM
    7 ... MINING_LARGE""")
        sys.exit(3)

    sigHandler = SignalHandler()
    projSvc = ProjectService(args.connectionstring)
    ctrlSvc = ControlService(args.connectionstring)

    # prepare project
    media = mediaString(projSvc)
    print("media = {}".format(media))
    registerScanposition(
        sigHandler, ctrlSvc, media,
        args.project, args.scanposition, args.registrationmode)

if __name__ == "__main__":
    main()
