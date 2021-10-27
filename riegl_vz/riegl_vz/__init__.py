import sys
from datetime import datetime
import numpy as np
from std_srvs.srv import (
    Trigger,
    SetBool
)
from sensor_msgs.msg import (
    PointCloud2,
)
from geometry_msgs.msg import (
    PoseStamped
)
from nav_msgs.msg import (
    Path,
    Odometry
)
from diagnostic_msgs.msg import (
    DiagnosticArray,
    DiagnosticStatus
)
from diagnostic_updater import Updater
from riegl_vz_interfaces.srv import (
    GetPointCloud,
    GetScanPoses,
    GetPose,
    SetPosition
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

_rieglVzWrapper = None

class RieglVzWrapper(Node):

    def __init__(self):
        super().__init__('riegl_vz')

        self._shutdownReq = False

        self.declare_parameter('hostname', 'H2222222')
        self.declare_parameter('working_dir', '/tmp/ros_riegl_vz')
        self.declare_parameter('ssh_user', 'user')
        self.declare_parameter('ssh_password', 'user')
        self.declare_parameter('project_name', '')
        self.declare_parameter('storage_media', 0)
        self.declare_parameter('scan_pattern', [30.0,130.0,0.04,0.0,360.0,0.5])
        self.declare_parameter('meas_program', 0)
        self.declare_parameter('scan_publish', True)
        self.declare_parameter('scan_publish_filter', '')
        self.declare_parameter('scan_publish_lod', 0)
        self.declare_parameter('scan_register', True)

        self.hostname = str(self.get_parameter('hostname').value)
        self.workingDir = str(self.get_parameter('working_dir').value)
        self.sshUser = str(self.get_parameter('ssh_user').value)
        self.sshPwd = str(self.get_parameter('ssh_password').value)
        self.projectName = str(self.get_parameter('project_name').value)
        self.get_logger().debug("hostname = {}".format(self.hostname))
        self.get_logger().debug("workingDir = {}".format(self.workingDir))
        self.get_logger().debug("sshUser = {}".format(self.sshUser))
        self.get_logger().debug("sshPwd = {}".format(self.sshPwd))
        self.get_logger().debug("projectName = {}".format(self.projectName))

        self.scanPublishFilter = str(self.get_parameter('scan_publish_filter').value)
        self.get_logger().debug("scanPublishFilter = {}".format(self.scanPublishFilter))
        self.scanPublishLOD = int(self.get_parameter('scan_publish_lod').value)
        self.get_logger().debug("scanPublishLOD = {}".format(self.scanPublishLOD))

        self.pointCloudPublisher = self.create_publisher(PointCloud2, 'pointcloud', 2)
        self.posePublisher = self.create_publisher(PoseStamped, 'pose', 10)
        self.pathPublisher = self.create_publisher(Path, 'path', 10)
        self.odomPublisher = self.create_publisher(Odometry, 'odom', 10)

        self._setProjectService = self.create_service(Trigger, 'set_project', self._setProjectCallback)
        self._scanService = self.create_service(Trigger, 'scan', self._scanCallback)
        self._getPointCloudService = self.create_service(GetPointCloud, 'get_pointcloud', self._getPointCloudCallback)
        self._setPositionService = self.create_service(SetPosition, 'set_position', self._setPositionCallback)
        self._getSopvService = self.create_service(GetPose, 'get_sopv', self._getSopvCallback)
        self._getVopService = self.create_service(GetPose, 'get_vop', self._getVopCallback)
        self._getPopService = self.create_service(GetPose, 'get_pop', self._getPopCallback)
        self._getScanPoses = self.create_service(GetScanPoses, 'get_scan_poses', self._getScanPosesCallback)
        self._stopService = self.create_service(Trigger, 'stop', self._stopCallback)
        self._trigStartStopService = self.create_service(Trigger, 'trig_start_stop', self._trigStartStopCallback)
        self._shutdownService = self.create_service(Trigger, 'shutdown', self._shutdownCallback)

        self._rieglVz = RieglVz(self)

        self.setProject("")
        self.get_logger().debug("init projectName = {}".format(self.projectName))

        self._statusUpdater = Updater(self)
        self._statusUpdater.setHardwareID('riegl_vz')
        self._statusUpdater.add("status", self.produceDiagnostics)

        self.get_logger().info("RIEGL VZ is ready, waiting for commands... (host = {}).".format(self.hostname))

    def produceDiagnostics(self, diag):
        status = self._rieglVz.getStatus()
        diag.summary(DiagnosticStatus.OK, "RIEGL VZ is " + status.opstate)
        diag.add('opstate', status.opstate)
        diag.add('progress', str(status.progress))
        return diag

    def setProject(self, projectName):
        now = datetime.now()
        if projectName == "":
            self.projectName = now.strftime("%y%m%d_%H%M%S")
        else:
            self.projectName = projectName
        self._rieglVz.resetPath()

    def loadProject(self, projectName):
        self.storageMedia = int(self.get_parameter('storage_media').value)
        ok = True
        if projectName == "" or self._rieglVz.checkProjectPath(self.projectName, self.storageMedia):
            ok = False
        else:
            self.projectName = projectName
        return ok

    def _setProjectCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        self.projectName = str(self.get_parameter('project_name').value)

        if self.projectName == "" or not self.loadProject(self.projectName):
            self.setProject(self.projectName)

        response.success = True
        response.message = self.projectName

        return response

    def scan(self):
        self.storageMedia = int(self.get_parameter('storage_media').value)
        scanPattern = self.get_parameter('scan_pattern').value
        self.scanPattern = ScanPattern()
        self.scanPattern.lineStart = scanPattern[0]
        self.scanPattern.lineStop = scanPattern[1]
        self.scanPattern.lineIncrement = scanPattern[2]
        self.scanPattern.frameStart = scanPattern[3]
        self.scanPattern.frameStop = scanPattern[4]
        self.scanPattern.frameIncrement = scanPattern[5]
        self.scanPattern.measProgram = int(self.get_parameter('meas_program').value)
        self.scanPublish = bool(self.get_parameter('scan_publish').value)
        self.scanPublishFilter = str(self.get_parameter('scan_publish_filter').value)
        self.scanPublishLOD = int(self.get_parameter('scan_publish_lod').value)
        self.scanRegister = bool(self.get_parameter('scan_register').value)

        self._scanposName = self._rieglVz.getNextScanpos(self.projectName, self.storageMedia)

        return self._rieglVz.scan(
            projectName = self.projectName,
            scanposName = self._scanposName,
            storageMedia = self.storageMedia,
            scanPattern = self.scanPattern,
            scanPublishFilter = self.scanPublishFilter,
            scanPublish = self.scanPublish,
            scanPublishLOD = self.scanPublishLOD,
            scanRegister = self.scanRegister,
            reflSearchSettings = None,
            captureImages = False,
            captureMode = 1,
            imageOverlap = 25)

    def _scanCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        if not self.scan():
            response.success = False
            response.message = "operation not available"
            return response

        response.success = True
        response.message = "success"

        self._statusUpdater.force_update()

        return response

    def getPointCloud(self, scanpos, pointcloud):
        ok, pointcloud = self._rieglVz.getPointCloud(scanpos, pointcloud)
        return ok, pointcloud

    def _getPointCloudCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        ok, response.pointcloud = self.getPointCloud(request.seq, response.pointcloud)
        if not ok:
            response.success = False
            response.message = "data not available"
            return response

        response.success = True
        response.message = "success"

        return response

    def setPosition(self, position):
        return self._rieglVz.setPosition(position)

    def _setPositionCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        self.setPosition(request.position)

        response.success = True
        response.message = "success"

        return response

    def getSopv(self):
        return self._rieglVz.getSopv()

    def _getSopvCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        ok, sopv = self.getSopv()
        if not ok:
            response.success = False
            response.message = "data not available"
            return response

        response.pose = sopv.pose
        response.success = True
        response.message = "success"

        return response

    def getAllSopv(self):
        return self._rieglVz.getAllSopv()

    def getVop(self):
        return self._rieglVz.getVop()

    def getPop(self):
        return self._rieglVz.getPop()

    def _getScanPosesCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        response.project = self.projectName

        ok, sopvs = self.getAllSopv()
        if not ok:
            response.success = False
            response.message = "data not available"
            return response
        for sopv in sopvs:
            response.scanposes.append(sopv)

        ok, vop = self.getVop()
        if not ok:
            response.success = False
            response.message = "data not available"
            return response
        response.vop = vop

        ok, pop = self.getPop()
        if ok:
            response.pop = pop
        else:
            response.pop = PoseStamped()

        response.success = True
        response.message = "success"

        return response

    def _getVopCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        ok, vop = self.getVop()
        if not ok:
            response.success = False
            response.message = "data not available"
            return response

        response.pose = vop
        response.success = True
        response.message = "success"

        return response

    def _getPopCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        ok, pop = self.getPop()
        if not ok:
            response.success = False
            response.message = "data not available"
            return response

        response.pose = pop
        response.success = True
        response.message = "success"

        return response

    def stop(self):
        self._rieglVz.stop()

    def _stopCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        self.stop()

        response.success = True
        response.message = "success"

        return response

    def trigStartStop(self):
        return self._rieglVz.trigStartStop()

    def _trigStartStopCallback(self, request, response):
        if self._shutdownReq is True:
            response.success = False
            response.message = "node is shutting down"
            return response

        if not self.trigStartStop():
            response.success = False
            response.message = "operation not available"
            return response

        response.success = True
        response.message = "success"

        return response

    def shutdown(self):
        self._shutdownReq = True
        self.stop()
        self._rieglVz.shutdown()

    def _shutdownCallback(self, request, response):
        self.shutdown()

        response.success = True
        response.message = "success"

        return response

def stop_node():
    if _rieglVzWrapper is not None:
        _rieglVzWrapper.stop()

    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    _rieglVzWrapper = RieglVzWrapper()
    try:
        rclpy.spin(_rieglVzWrapper)
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
