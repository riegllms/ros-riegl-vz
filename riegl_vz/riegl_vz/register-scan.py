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

finishedWithError = False
def registerScanposition(sigHandler, ctrlSvc, media, project, scanposition, mode, waitUntilFinished = True):
    """Register scanposition. Blocks until registration is finished."""
    taskId = None

    if waitUntilFinished:
        finishedEvent = Event()
        def onBackgroundTaskRemoved(arg0):
            global finishedWithError
            obj = json.loads(arg0)
            if obj.get('id') == taskId:
                if obj.get('state') != 'finished':
                    finishedWithError = True
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
            return 1

        sigcon.disconnect()
    else:
        ctrlSvc.addRegistrationTask(media, project, scanposition, mode)

    if finishedWithError:
        return 1

    return 0

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
    parser.add_argument('--wait-until-finished',  action='store_true',
        help='block until scan registration has finished')
    args = parser.parse_args()

    # verify command line options
    if not args.project:
        print("No project name specified.")
        sys.exit(2)
    if not args.scanposition:
        print("No scanposition name specified.")
        sys.exit(3)
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
        sys.exit(4)

    sigHandler = SignalHandler()
    projSvc = ProjectService(args.connectionstring)
    ctrlSvc = ControlService(args.connectionstring)

    # prepare project
    media = mediaString(projSvc)
    print("media = {}".format(media))
    return registerScanposition(
        sigHandler, ctrlSvc, media,
        args.project, args.scanposition, args.registrationmode, args.wait_until_finished)

if __name__ == "__main__":
    import sys
    sys.exit(main())
