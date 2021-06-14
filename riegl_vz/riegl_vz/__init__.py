import sys
from datetime import datetime
import numpy as np
from std_srvs.srv import (
    Trigger,
    SetBool
)
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from .riegl_vz import (
    ScanPattern,
    RieglVz
)
from .utils import (
    SubProcess
)

rieglVzWrapper = None

class RieglVzWrapper(Node):

    def __init__(self):
        super().__init__('riegl_vz')

        self.shutdownReq = False
        self.subproc = None

        self.declare_parameter('hostname', 'H2222222')
        self.declare_parameter('working_dir', '~/.ros_riegl_vz')
        self.declare_parameter('ssh_user', 'user')
        self.declare_parameter('ssh_password', 'user')
        self.declare_parameter('project_name', '')
        self.declare_parameter('scan_publish', True)
        self.declare_parameter('scan_pattern', [30.0,130.0,0.04,0.0,360.0,0.04])
        self.declare_parameter('meas_program', 0)
        self.declare_parameter('msm', 1)

        self.hostname = str(self.get_parameter('hostname').value)
        self.connectionString = self.hostname + ":20000"
        self.workingDir = str(self.get_parameter('working_dir').value)
        self.sshUser = str(self.get_parameter('ssh_user').value)
        self.sshPwd = str(self.get_parameter('ssh_password').value)
        self.projectName = str(self.get_parameter('project_name').value)
        self.scanposName = ''
        self.scanPublish = bool(self.get_parameter('scan_publish').value)
        scanPattern = self.get_parameter('scan_pattern').value
        self.scanPattern = ScanPattern()
        self.scanPattern.lineStart = float(scanPattern[0])
        self.scanPattern.lineStop = scanPattern[1]
        self.scanPattern.lineIncr = scanPattern[2]
        self.scanPattern.frameStart = scanPattern[3]
        self.scanPattern.frameStop = scanPattern[4]
        self.scanPattern.frameIncr = scanPattern[5]
        self.measProgram = int(self.get_parameter('meas_program').value)
        self.msm = int(self.get_parameter('msm').value)

        self.scanService = self.create_service(Trigger, 'scan', self.scanCallback)
        self.isBusyService = self.create_service(SetBool, 'is_busy', self.isBusyCallback)
        self.stopService = self.create_service(Trigger, 'stop', self.stopCallback)
        self.shutdownService = self.create_service(Trigger, 'shutdown', self.shutdownCallback)

        self.rieglVz = RieglVz(self.connectionString, self.workingDir, self.get_logger())

        self.get_logger().info("RIEGL VZ is now started, ready to get commands. (host = {}).".format(self.connectionString))

    def scanCallback(self, request, response):
        if self.shutdownReq is True:
            return {"success": False, "message": "RIEGL VZ is shutting down"}

        now = datetime.now()
        if not self.projectName:
            self.projectName = now.strftime("%y%m%d_%H%M%S")
        self.scanposName = now.strftime("%y%m%d_%H%M%S")

        self.subproc = self.rieglVz.acquireData(
            projectName = self.projectName,
            scanposName = self.scanposName,
            scanPattern = self.scanPattern,
            reflSearchSettings = None,
            createRdbx = True,
            block = False,
            rdbxLineStep = self.msm,
            rdbxEchoStep = self.msm,
            captureImages = False,
            captureMode = 1,
            imageOverlap = 25)
        response.success = True
        response.message = "success"
        return response

    def isBusyCallback(self, request):
        if self.subproc is not None:
            block = request.data
            if self.subproc.waitFor("Execution failed.", block):
                self.subproc = None
        else:
            return {"success": False, "message": "RIEGL VZ is not busy"}
        return {"success": True, "message": "RIEGL VZ is busy"}

    def stop(self):
        if self.subproc is not None:
            self.subproc.cancel();
            self.rieglVz.stop()

    def stopCallback(self, request):
        stop()
        return {"success": True, "message": "RIEGL VZ has been stopped"}

    def shutdownCallback(self, request):
        self.shutdownReq = True
        stop()
        self.rieglVz.shutdown()
        return {"success": True, "message": "RIEGL VZ scanner is shutting down"}

def stop_node():
    if rieglVzWrapper is not None:
        rieglVzWrapper.stop()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    rieglVzWrapper = RieglVzWrapper()
    try:
        rclpy.spin(rieglVzWrapper)
    except KeyboardInterrupt:
        pass
    finally:
        stop_node()

if __name__ == "__main__":
    try:
        signal.signal(signal.SIGINT, stop_node)
        signal.signal(signal.SIGTERM, stop_node)
        main()
    except:
        stop_node()
