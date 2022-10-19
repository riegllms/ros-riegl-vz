#!/bin/env python3

import sys
import argparse
import ssh

def main():
    parser = argparse.ArgumentParser(description='Perform a coarse registration of a specified scan position.')
    parser.add_argument('--hostname',
        default='127.0.0.1',
        help='hostname or ip address of the scanner')
    parser.add_argument('--sshuser',
        default='user',
        help='ssh user name (default \'user\')')
    parser.add_argument('--sshpwd',
        default='user',
        help='ssh password (default \'user\')')
    parser.add_argument('--project-path',
        help='project path')
    parser.add_argument('--scanposition',
        help='scanposition name')
    parser.add_argument('--scanposition-prev',
        help='previous scanposition name')
    parser.add_argument('--scanposition-pre-prev',
        help='pre-previous scanposition name')
    parser.add_argument('--rdbx',
        default='/tmp/fastpos.rdbx',
        help='rdbx file path (default \'/tmp/fastpos.rdbx\')')
    args = parser.parse_args()

    if not args.project_path:
        print("No project path specified.")
        sys.exit(1)

    if not args.scanposition:
        print("No scan position name specified.")
        sys.exit(2)

    rcl = ssh.RemoteClient(host=args.hostname, user=args.sshuser, password=args.sshpwd)
    cmd = [ "/opt/riegl/robot-tools/coarse-registration" ]
    if args.scanposition_prev:
        cmd.extend(['--prev', args.scanposition_prev])
    if args.scanposition_pre_prev:
        cmd.extend(['--pre-prev', args.scanposition_pre_prev])
    outFile = args.project_path + '/' + args.scanposition + '.SCNPOS/robot_fast_pose.sopv'
    cmd.extend([args.project_path, args.scanposition, args.rdbx, '>' + outFile])
    rc, response = rcl.executeCommand(' '.join(cmd))
    rcl.disconnect()

    return rc

if __name__ == "__main__":
    sys.exit(main())
