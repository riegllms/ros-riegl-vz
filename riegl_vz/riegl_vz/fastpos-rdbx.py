#!/bin/env python3

import sys
import argparse
import time
from vzi_services.dataprocservice import DataprocService
import ssh

def main():
    parser = argparse.ArgumentParser(description='Create a RDBX file from a RXP data stream (without MTA resolution).')
    parser.add_argument('--connectionstring',
        default='127.0.0.1:20000',
        help='address of scanner services')
    parser.add_argument('--sshuser',
        default='user',
        help='ssh user name (default \'user\')')
    parser.add_argument('--sshpwd',
        default='user',
        help='ssh password (default \'user\')')
    parser.add_argument('--rdbx',
        default='/tmp/fastpos.rdbx',
        help='rdbx file path (default \'/tmp/fastpos.rdbx\')')
    args = parser.parse_args()

    procSvc = DataprocService(args.connectionstring)

    cnt = 0
    while not procSvc.isRunning() and cnt < 100:
        time.sleep(0.1)
        cnt += 1
    if cnt >= 100:
        print("Timeout waiting for scan data acquisition!")
        sys.exit(1)

    hostname = args.connectionstring.split(':')[0]

    rcl = ssh.RemoteClient(host=hostname, user=args.sshuser, password=args.sshpwd)
    cmd = ["/opt/riegl/robot-tools/fastposrdbx", " rdtp://127.0.0.1/ACTUAL?type=meas", args.rdbx]
    rc, response = rcl.executeCommand(' '.join(cmd))
    rcl.disconnect()

    return rc

if __name__ == "__main__":
    sys.exit(main())
