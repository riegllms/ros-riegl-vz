import sys
from datetime import datetime
import numpy as np

from std_srvs.srv import (
    Trigger,
    SetBool
)
from sensor_msgs.msg import (
    PointCloud2,
    NavSatFix,
    Imu,
    MagneticField
)
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped
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
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from riegl_vz_interfaces.msg import (
    Voxels
)
from riegl_vz_interfaces.srv import (
    GetPointCloud,
    GetVoxels,
    GetScanPoses,
    GetTpl,
    GetPose,
    SetPosition,
    SetPose,
    GetList,
    TransformCoord,
    Command,
    MoveFile
)
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from .riegl_vz import (
    ScanPattern,
    RieglVz
)
from .pose import (
    getTransformFromArray
)
from .utils import (
    SubProcess
)

_rieglVzWrapper = None

class RieglVzWrapper(Node):

    def __init__(self):
        super().__init__('riegl_vz')

        self._stopNodeReq = False
        self._shutdownReq = False

        self.declare_parameter('hostname', 'H2222222')
        self.declare_parameter('working_dir', '/tmp/ros_riegl_vz')
        self.declare_parameter('ssh_user', 'user')
        self.declare_parameter('ssh_password', 'user')
        self.declare_parameter('project_name', '')
        self.declare_parameter('scanpos_name', '')
        self.declare_parameter('storage_media', 0)
        self.declare_parameter('scan_pattern', [30.0,130.0,0.1,0.0,360.0,0.5])
        self.declare_parameter('scan_pattern_name', '')
        self.declare_parameter('meas_program', 0)
        self.declare_parameter('scan_publish', True)
        self.declare_parameter('voxel_publish', False)
        self.declare_parameter('scan_publish_filter', '')
        self.declare_parameter('scan_publish_lod', 0)
        self.declare_parameter('scan_register', True)
        self.declare_parameter('scan_registration_mode', 1)
        self.declare_parameter('pose_publish', True)
        self.declare_parameter('pose_publish_fast', False)
        self.declare_parameter('reflector_search', False)
        self.declare_parameter('reflector_search_models', '')
        self.declare_parameter('reflector_search_limits', [0.0, 10000.0])
        self.declare_parameter('control_points_csv_file_remote', '')
        self.declare_parameter('control_points_csv_file_local', '')
        self.declare_parameter('control_points_coord_system', '')
        self.declare_parameter('image_capture', 0)
        self.declare_parameter('image_capture_mode', 1)
        self.declare_parameter('image_capture_overlap', 25)
        self.declare_parameter('set_pose_topic', '')
        self.declare_parameter('relative_pose_mode', False)
        self.declare_parameter('robot_scanner_mounting', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('robot_project_frame_id', '')
        self.declare_parameter('robot_project_transform', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('imu_data_publish', False)
        self.declare_parameter('imu_index', 0)
        self.hostname = str(self.get_parameter('hostname').value)
        self.workingDir = str(self.get_parameter('working_dir').value)
        self.sshUser = str(self.get_parameter('ssh_user').value)
        self.sshPwd = str(self.get_parameter('ssh_password').value)
        self.projectName = str(self.get_parameter('project_name').value)
        self.scanPosPrefix = str(self.get_parameter('scanpos_name').value)
        self.storageMedia = int(self.get_parameter('storage_media').value)
        self.get_logger().info("hostname = {}".format(self.hostname))
        self.get_logger().info("workingDir = {}".format(self.workingDir))
        self.get_logger().info("sshUser = {}".format(self.sshUser))
        self.get_logger().info("sshPwd = {}".format(self.sshPwd))
        self.get_logger().info("projectName = {}".format(self.projectName))
        self.get_logger().info("scanPosName = '{}'".format(self.scanPosPrefix))
        self.get_logger().info("storageMedia = {}".format(self.storageMedia))

        self.scanPublishFilter = str(self.get_parameter('scan_publish_filter').value)
        self.get_logger().info("scanPublishFilter = {}".format(self.scanPublishFilter))
        self.scanPublishLOD = int(self.get_parameter('scan_publish_lod').value)
        self.get_logger().info("scanPublishLOD = {}".format(self.scanPublishLOD))

        self.imuDataPublish = bool(self.get_parameter('imu_data_publish').value)
        self.get_logger().info("imuDataPublish = {}".format(self.imuDataPublish))
        self.imuSelect = int(self.get_parameter('imu_index').value)
        self.get_logger().info("imuIndex = {}".format(self.imuSelect))

        # create topics..
        self.pointCloudPublisher = self.create_publisher(PointCloud2, 'pointcloud', 2)
        self.voxelsPublisher = self.create_publisher(Voxels, 'voxels', 2)
        self.posePublisher = self.create_publisher(PoseStamped, 'pose', 10)
        self.poseFastPublisher = self.create_publisher(PoseStamped, 'pose_fast', 10)
        self.pathPublisher = self.create_publisher(Path, 'path', 10)
        self.odomPublisher = self.create_publisher(Odometry, 'odom', 10)
        self.gnssFixPublisher = self.create_publisher(NavSatFix, 'gnss', 10)
        self.imuPublisher= self.create_publisher(Imu,'imu_data',10)
        self.magnetPublisher= self.create_publisher(MagneticField,'magnetic_field',10)
        self.setPoseTopic = str(self.get_parameter('set_pose_topic').value)
        if self.setPoseTopic != "":
            self.get_logger().info("setPoseTopic = {}".format(self.setPoseTopic))
            try:
                self.setPoseSubscription = self.create_subscription(
                    PoseWithCovarianceStamped,
                    self.setPoseTopic,
                    self._setPoseTopicCallback,
                    10)
            except Exception as e:
                self.get_logger().error("Set pose topic subscription failed!")

        # tf2 message broadcaster..
        self.transformBroadcaster = TransformBroadcaster(self)
        self._broadcastTfRobotProjectTransform()

        # tf2 listener..
        self.transformBuffer = Buffer()
        self.transformListener = TransformListener(self.transformBuffer, self)

        # create documented services..
        self._setProjectService = self.create_service(Trigger, 'set_project', self._setProjectCallback)
        self._setPositionService = self.create_service(SetPosition, 'set_position', self._setPositionCallback)
        self._setPoseService = self.create_service(SetPose, 'set_pose', self._setPoseCallback)
        self._scanService = self.create_service(Trigger, 'scan', self._scanCallback)
        self._getScanPoses = self.create_service(GetScanPoses, 'get_scan_poses', self._getScanPosesCallback)
        self._stopService = self.create_service(Trigger, 'stop', self._stopCallback)
        self._shutdownService = self.create_service(Trigger, 'shutdown', self._shutdownCallback)

        # create undocumented services..
        self._getPointCloudService = self.create_service(GetPointCloud, 'get_pointcloud', self._getPointCloudCallback)
        self._getVoxelsService = self.create_service(GetVoxels, 'get_voxels', self._getVoxelsCallback)
        self._getSopvService = self.create_service(GetPose, 'get_sopv', self._getSopvCallback)
        self._getVopService = self.create_service(GetPose, 'get_vop', self._getVopCallback)
        self._getPopService = self.create_service(GetPose, 'get_pop', self._getPopCallback)
        self._getTplService = self.create_service(GetTpl, 'get_tpl', self._getTplCallback)
        self._testService = self.create_service(Trigger, 'test', self._testCallback)
        self._trigStartStopService = self.create_service(Trigger, 'trig_start_stop', self._trigStartStopCallback)
        self._getScanPatterns = self.create_service(GetList, 'get_scan_patterns', self._getScanPatternsCallback)
        self._getReflectorModels = self.create_service(GetList, 'get_reflector_models', self._getReflectorModelsCallback)
        self._transformGeoCoordService = self.create_service(TransformCoord, 'transform_geo_coord', self._transformGeoCoordCallback)
        self._executComand = self.create_service(Command, 'execute_command', self._commandCallback)
        self._download = self.create_service(MoveFile, 'download', self._downloadCallback)
        self._upload = self.create_service(MoveFile, 'upload', self._uploadCallback)

        self._rieglVz = RieglVz(self)

        self._scanposition = '0'
        self.projectValid = False
        self.imuDataConnected = False

        self._statusUpdater = Updater(self)
        self._statusUpdater.setHardwareID('riegl_vz')
        self._statusUpdater.add('scanner', self._produceScannerDiagnostics)
        self._statusUpdater.add('memory', self._produceMemoryDiagnostics)
        self._statusUpdater.add('gnss', self._produceGnssDiagnostics)
        self._statusUpdater.add('errors', self._produceErrorDiagnostics)
        self._statusUpdater.add('camera', self._produceCameraDiagnostics)

        self._gnssFixTimer = self.create_timer(1.0, self._publishGnssFix)

        self.get_logger().info("RIEGL VZ node is started... (host = {}).".format(self.hostname))


    def _broadcastTfRobotProjectTransform(self):
        self.robotProjectFrameId = str(self.get_parameter('robot_project_frame_id').value)
        if self.robotProjectFrameId != "":
            self.get_logger().info("robot_project_frame_id = {}".format(self.robotProjectFrameId))
            self.robotProjectTransform = self.get_parameter('robot_project_transform').value
            self._logger.info("robot project transform = x: {0}, y: {1}, z: {2}, roll: {3}, pitch: {4}, yaw: {5}"
                .format(
                    self.robotProjectTransform[0],
                    self.robotProjectTransform[1],
                    self.robotProjectTransform[2],
                    self.robotProjectTransform[3],
                    self.robotProjectTransform[4],
                    self.robotProjectTransform[5]))
            self.transformBroadcaster.sendTransform(
                getTransformFromArray(
                    self.get_clock().now(),
                    self.robotProjectFrameId,
                    'riegl_vz_prcs',
                    self.robotProjectTransform
                ))

    def _produceScannerDiagnostics(self, diag):
        status = self._rieglVz.getScannerStatus()

        err = DiagnosticStatus.OK
        message = 'ok'
        if status.opstate == 'unavailable':
            err = DiagnosticStatus.WARN
            message = 'N/A'
        else:
            if status.err:
                err = DiagnosticStatus.ERROR
                message = 'com error'
            if self.imuDataPublish and not self.imuDataConnected: # check if scanner is online
                self._rieglVz.initImu()
                self.imuDataConnected = True

        if status.taskErrors != '':
            err = DiagnosticStatus.ERROR
            message = 'background task error(s)'

        diag.summary(err, message)
        diag.add('opstate', status.opstate)
        if status.opstate != 'unavailable':
            diag.add('active_task', status.activeTask)
            diag.add('task_error', status.taskErrors)
            diag.add('progress', str(status.progress))
            diag.add('scan_position', self._scanposition)
            diag.add('laser', 'on' if status.laserOn else 'off')

        return diag

    def _produceMemoryDiagnostics(self, diag):
        status = self._rieglVz.getMemoryStatus()

        if not status.valid:
            diag.summary(DiagnosticStatus.WARN, 'N/A')
            return diag

        err = DiagnosticStatus.OK
        message = 'ok'
        if status.err:
            err = DiagnosticStatus.ERROR
            message = 'com error'
        elif status.memUsage >= 90.0:
            err = DiagnosticStatus.WARN
            message = 'memory almost full'
        elif status.memUsage >= 99.0:
            err = DiagnosticStatus.ERROR
            message = 'memory full'

        diag.summary(err, message)
        #diag.add('mem_total_gb', str(status.memTotalGB))
        diag.add('mem_free_gb', str(status.memFreeGB))
        diag.add('mem_usage', str(status.memUsage))

        return diag

    def _produceGnssDiagnostics(self, diag):
        status = self._rieglVz.getGnssStatus()

        if not status.valid:
            diag.summary(DiagnosticStatus.WARN, 'N/A')
            return diag

        err = DiagnosticStatus.OK
        message = 'ok'
        if status.err:
            err = DiagnosticStatus.ERROR
            message = 'com error'

        diag.summary(err, message)
        diag.add('gnss_ena', str(status.enabled))
        diag.add('gnss_fix', str(status.fix))
        diag.add('gnss_num_sat', str(status.numSat))
        return diag

    def _produceErrorDiagnostics(self, diag):
        status = self._rieglVz.getErrorStatus()

        if not status.valid:
            diag.summary(DiagnosticStatus.WARN, 'N/A')
            return diag

        err = DiagnosticStatus.OK
        message = 'ok'
        if status.err:
            err = DiagnosticStatus.ERROR
            message = 'com error'
        elif status.numErrors > 0:
            err = DiagnosticStatus.ERROR
            message = 'system error(s)'
        elif status.numWarnings > 0:
            err = DiagnosticStatus.WARN
            message = 'system warning(s)'

        diag.summary(err, message)
        diag.add('warn_num', str(status.numWarnings))
        diag.add('err_num', str(status.numErrors))
        return diag

    def _produceCameraDiagnostics(self, diag):
        status = self._rieglVz.getCameraStatus()

        if not status.valid:
            diag.summary(DiagnosticStatus.WARN, 'N/A')
            return diag

        err = DiagnosticStatus.OK
        message = 'ok'
        if not status.avail:
            err = DiagnosticStatus.WARN
            message = 'no camera'
        elif status.err:
            err = DiagnosticStatus.ERROR
            message = 'com error'

        diag.summary(err, message)
        diag.add('cam_avail', str(status.avail))
        return diag

    def _publishGnssFix(self):
            self._rieglVz.publishGnssFix()

    def _setResponseStatus(self, response, success, message):
        response.success = success
        response.message = message
        return success, response

    def _setResponseSuccess(self, response):
        return self._setResponseStatus(response, True, 'success')[1]

    def _setResponseExecError(self, response):
        self._logger.error("Service request command execution error!")
        return self._setResponseStatus(response, False, 'command execution error')[1]

    def _setResponseException(self, response, e):
        self._logger.error("Service request command exception: " + str(e))
        return self._setResponseStatus(response, False, 'command execution error')[1]

    def _checkExecConditions(self):
        success = True
        message = 'success'
        if not self._rieglVz.isScannerAvailable() or self._shutdownReq:
            success = False
            message = 'device not available'
            self._logger.warning("Device is not available.")
        return success, message

    def _setProjectName(self, projectName):
        if projectName == '':
            now = datetime.now()
            self.projectName = now.strftime("%y%m%d_%H%M%S")
        else:
            self.projectName = projectName

    def _createProject(self, projectName):
        self._setProjectName(projectName)
        self.storageMedia = int(self.get_parameter('storage_media').value)
        self.get_logger().info("storage media = {}".format(self.storageMedia))
        ok = True
        if not self._rieglVz.createProject(self.projectName, self.storageMedia):
            ok = False
        return ok

    def _loadProject(self, projectName):
        self.storageMedia = int(self.get_parameter('storage_media').value)
        self.get_logger().info("storage media = {}".format(self.storageMedia))
        self.scanRegister = bool(self.get_parameter('scan_register').value)
        self.get_logger().info("scan register = {}".format(self.scanRegister))
        self.posePublish = bool(self.get_parameter('pose_publish').value)
        self.get_logger().info("pose publish = {}".format(self.posePublish))
        self.posePublishFast = bool(self.get_parameter('pose_publish_fast').value)
        self.get_logger().info("pose publish fast = {}".format(self.posePublish))
        ok = True
        if projectName == '' or not self._rieglVz.loadProject(self.projectName, self.storageMedia, self.scanRegister and (self.posePublish or self.posePublishFast)):
            ok = False
        else:
            self.projectName = projectName
        return ok

    def setProject(self, projectName):
        ok = True

        self.scanPosPrefix = str(self.get_parameter('scanpos_name').value)
        self.get_logger().info("scanpos prefix = {}".format(self.scanPosPrefix))
        self._rieglVz.setScanPosPrefix(self.scanPosPrefix)

        if not self._loadProject(self.projectName):
            ok = self._createProject(self.projectName)

        if ok:
            self.projectValid = True
            self.cpsCsvFile = ""
            self.cpsCsvFileRemote = str(self.get_parameter('control_points_csv_file_remote').value)
            self.get_logger().info("remote control points CSV file  = {}".format(self.cpsCsvFileRemote))
            self.cpsCsvFileLocal = str(self.get_parameter('control_points_csv_file_local').value)
            self.get_logger().info("local control points CSV file  = {}".format(self.cpsCsvFileLocal))
            if len(self.cpsCsvFileRemote) > 0 or len(self.cpsCsvFileLocal):
                self.cpsCsvCoordSystem = str(self.get_parameter('control_points_coord_system').value)
                self.get_logger().info("control points coord system = {}".format(self.cpsCsvCoordSystem))
                self._rieglVz.setProjectControlPoints(self.cpsCsvCoordSystem, self.cpsCsvFileRemote, self.cpsCsvFileLocal)

        return ok

    def _setProjectCallback(self, request, response):
        self.get_logger().info("Service Request: set_project")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            self.projectName = str(self.get_parameter('project_name').value)
            self.get_logger().info("project name = {}".format(self.projectName))
            self.storageMedia = int(self.get_parameter('storage_media').value)
            self.get_logger().info("storage media = {}".format(self.storageMedia))

            if not self.setProject(self.projectName):
                self._setResponseExecError(response)
                return response

            self._scanposition = self._rieglVz.getCurrentScanpos(self.projectName, self.storageMedia)
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def scan(self):
        self.storageMedia = int(self.get_parameter('storage_media').value)
        scanPatternName = self.get_parameter('scan_pattern_name').value
        if scanPatternName != '':
            self.get_logger().info("scan pattern name = {}".format(scanPatternName))
            ok, self.scanPattern = self._rieglVz.getScanPattern(scanPatternName)
            if not ok:
                scanPatternName = ''
        if scanPatternName == '':
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
        self.voxelPublish = bool(self.get_parameter('voxel_publish').value)
        self.scanRegister = bool(self.get_parameter('scan_register').value)
        self.scanRegistrationMode = int(self.get_parameter('scan_registration_mode').value)
        self.posePublish = bool(self.get_parameter('pose_publish').value)
        self.posePublishFast = bool(self.get_parameter('pose_publish_fast').value)
        self.reflSearchSettings = None
        self.reflSearch = bool(self.get_parameter('reflector_search').value)
        reflSearchModels = str(self.get_parameter('reflector_search_models').value)
        reflSearchLimits = self.get_parameter('reflector_search_limits').value
        if self.reflSearch and (len(reflSearchModels) > 0):
            self.reflSearchSettings = {
                'searchMode': 'model',
                'searchModels': [x.strip() for x in reflSearchModels.split(',')],
                'searchMinRange': reflSearchLimits[0],
                'searchMaxRange': reflSearchLimits[1]
            }
        self.imageCapture = int(self.get_parameter('image_capture').value)
        self.imageCaptureMode = int(self.get_parameter('image_capture_mode').value)
        self.imageCaptureOverlap = int(self.get_parameter('image_capture_overlap').value)

        if not self.projectValid:
            self.setProject(self.projectName)

        self._scanposition = self._rieglVz.getNextScanpos(self.projectName, self.storageMedia)

        return self._rieglVz.scan(
            projectName = self.projectName,
            scanposition = self._scanposition,
            storageMedia = self.storageMedia,
            scanPattern = self.scanPattern,
            scanPublishFilter = self.scanPublishFilter,
            scanPublish = self.scanPublish,
            scanPublishLOD = self.scanPublishLOD,
            voxelPublish = self.voxelPublish,
            scanRegister = self.scanRegister,
            scanRegistrationMode = self.scanRegistrationMode,
            posePublish = self.posePublish,
            posePublishFast = self.posePublishFast,
            reflSearchSettings = self.reflSearchSettings if self.reflSearch else None,
            captureImages = self.imageCapture,
            captureMode = self.imageCaptureMode,
            imageOverlap = self.imageCaptureOverlap)

    def _scanCallback(self, request, response):
        self.get_logger().info("Service Request: scan")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            if self._rieglVz.getScannerOpstate() != 'waiting':
                self._setResponseStatus(response, False, 'device is busy')
                self._logger.warning("Device is busy at the moument.")
                return response

            if not self.scan():
                self._setResponseExecError(response)
                return response

                self._statusUpdater.force_update
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def getPointCloud(self, scanpos, pointcloud):
        if not self.projectValid:
            self.setProject(self.projectName)
        if scanpos == 0:
            scanposition = self._rieglVz.getCurrentScanpos(self.projectName, self.storageMedia)
        else:
            scanposition = str(scanpos)
        return self._rieglVz.getPointCloud(scanposition, pointcloud)

    def _getPointCloudCallback(self, request, response):
        self.get_logger().info("Service Request: get_pointcloud")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, response.pointcloud = self.getPointCloud(request.seq, response.pointcloud)
            if not ok:
                self._setResponseExecError(response)
                return response
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def getVoxels(self, scanpos, voxels):
        if scanpos == 0:
            scanposition = self._rieglVz.getCurrentScanpos(self.projectName, self.storageMedia)
        else:
            scanposition = str(scanpos)
        ok, voxels = self._rieglVz.getVoxels(voxels, scanposition, False)
        return ok, voxels

    def _getVoxelsCallback(self, request, response):
        self.get_logger().info("Service Request: get_voxels")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, response.voxels = self.getVoxels(request.seq, response.voxels)
            if not ok:
                self._setResponseExecError(response)
                return response
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def setPosition(self, position, covariance):
        return self._rieglVz.setPosition(position, covariance)

    def _setPositionCallback(self, request, response):
        self.get_logger().info("Service Request: set_position")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            self.setPosition(request.position, request.covariance)
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def setPose(self, pose, isRelative, mountingPose):
        return self._rieglVz.setPose(pose, isRelative, mountingPose)

    def _setPoseCallback(self, request, response):
        self.get_logger().info("Service Request: set_pose")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            self.relativePoseMode = bool(self.get_parameter('relative_pose_mode').value)
            self.robotScannerMounting = self.get_parameter('robot_scanner_mounting').value
            self.setPose(request.pose, self.relativePoseMode, self.robotScannerMounting)
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def _setPoseTopicCallback(self, pose):
        self.get_logger().info("Topic Callback: set_pose")
        try:
            self.relativePoseMode = bool(self.get_parameter('relative_pose_mode').value)
            self.robotScannerMounting = self.get_parameter('robot_scanner_mounting').value
            self.setPose(pose, self.relativePoseMode, self.robotScannerMounting)
        except Exception as e:
            pass

    def getSopv(self):
        return self._rieglVz.getSopv()

    def _getSopvCallback(self, request, response):
        self.get_logger().info("Service Request: get_sopv")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, sopv = self.getSopv()
            if not ok:
                self._setResponseExecError(response)
                return response

            response.pose = sopv.pose
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def getAllSopv(self):
        return self._rieglVz.getAllSopv()

    def getVop(self):
        return self._rieglVz.getVop()

    def getPop(self):
        return self._rieglVz.getPop()

    def _getScanPosesCallback(self, request, response):
        self.get_logger().info("Service Request: get_scan_poses")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            response.project = self.projectName

            ok, sopvs = self.getAllSopv()
            if not ok:
                self._setResponseExecError(response)
                return response

            for sopv in sopvs:
                response.scanposes.append(sopv)

            ok, vop = self.getVop()
            if not ok:
                self._setResponseExecError(response)
                return response

            response.vop = vop

            ok, pop = self.getPop()
            if ok:
                response.pop = pop
            else:
                response.pop = PoseStamped()
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def _getVopCallback(self, request, response):
        self.get_logger().info("Service Request: get_vop")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, vop = self.getVop()
            if not ok:
                self._setResponseExecError(response)
                return response

            response.pose = vop
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def _getPopCallback(self, request, response):
        self.get_logger().info("Service Request: get_pop")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, pop = self.getPop()
            if not ok:
                self._setResponseExecError(response)
                return response

            response.pose = pop
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def stop(self):
        self._rieglVz.stop()

    def _stopCallback(self, request, response):
        self.get_logger().info("Service Request: stop")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            self.stop()
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def trigStartStop(self):
        return self._rieglVz.trigStartStop()

    def _trigStartStopCallback(self, request, response):
        self.get_logger().info("Service Request: trig_start_stop")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            if not self.trigStartStop():
                self._setResponseExecError(response)
                return response
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def getScanPatterns(self):
        return self._rieglVz.getScanPatterns()

    def _getScanPatternsCallback(self, request, response):
        self.get_logger().info("Service Request: get_scan_patterns")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, patterns = self.getScanPatterns()
            if not ok:
                self._setResponseExecError(response)
                return response

            for pattern in patterns:
                response.list.append(pattern)
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def getReflectorModels(self):
        return self._rieglVz.getReflectorModels()

    def _getReflectorModelsCallback(self, request, response):
        self.get_logger().info("Service Request: get_reflector_models")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, models = self.getReflectorModels()
            if not ok:
                self._setResponseExecError(response)
                return response

            for model in models:
                response.list.append(model)
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def getTpl(self, scanpos):
        if not self.projectValid:
            self.setProject(self.projectName)
        if scanpos == 0:
            scanposition = self._rieglVz.getCurrentScanpos(self.projectName, self.storageMedia)
        else:
            scanposition = str(scanpos)
        return self._rieglVz.getTpl(scanposition)

    def _getTplCallback(self, request, response):
        self.get_logger().info("Service Request: get_tpl")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, tpl = self.getTpl(request.seq)
            if not ok:
                self._setResponseExecError(response)
                return response

            for tie_point in tpl:
                response.tp_count+=1
                response.tpl.append(tie_point)
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def test(self):
        self.storageMedia = int(self.get_parameter('storage_media').value)
        self.scanPattern: ScanPattern = ScanPattern()
        self.scanPattern.lineStart = 30.0
        self.scanPattern.lineStop = 130.0
        self.scanPattern.lineIncrement = 0.1
        self.scanPattern.frameStart = 0.0
        self.scanPattern.frameStop = 360.0
        self.scanPattern.frameIncrement = 0.5
        self.scanPattern.measProgram = 3
        self.scanPublish = False
        self.voxelPublish = False
        self.scanRegister = False
        self.posePublish = False
        self.posePublishFast = False
        self.reflSearchSettings = None
        self.reflSearch = False
        self.imageCapture = 0

        if not self.projectValid:
            self.setProject(self.projectName)

        self._scanposition = 'Test'

        return self._rieglVz.scan(
            projectName = self.projectName,
            scanposition = self._scanposition,
            storageMedia = self.storageMedia,
            scanPattern = self.scanPattern,
            scanPublish = self.scanPublish,
            voxelPublish = self.voxelPublish,
            scanRegister = self.scanRegister,
            posePublish = self.posePublish,
            posePublishFast = self.posePublishFast,
            reflSearchSettings = self.reflSearchSettings,
            captureImages = self.imageCapture)

    def _testCallback(self, request, response):
        self.get_logger().info("Service Request: test")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            if self._rieglVz.getScannerOpstate() != 'waiting':
                self._setResponseStatus(response, False, 'device is busy')
                self._logger.warning("Device is busy at the moument.")
                return response

            if not self.test():
                self._setResponseExecError(response)
                return response

                self._statusUpdater.force_update
        except Exception as e:
            self._setResponseException(response, e)

        return response

    def _transformGeoCoordCallback(self, request, response):
        self.get_logger().info("Service Request: transform_geo_coord")
        try:
            if not self._setResponseStatus(response, *self._checkExecConditions())[0]:
                return response

            ok, coord1, coord2, coord3 = self._rieglVz.transformGeoCoordinate(request.src_cs, request.dst_cs, request.coord1, request.coord2, request.coord3)
            if not ok:
                self._setResponseExecError(response)
                return response

            response.coord1 = coord1
            response.coord2 = coord2
            response.coord3 = coord3

        except Exception as e:
            self._setResponseException(response, e)

        return response

    def _commandCallback(self, request, response):
        self.get_logger().info("Service Request: execute_command")
        command=request.command
        response.success=True
        response.message='ok'
        status = self._rieglVz.getScannerStatus()
        if status.opstate != 'unavailable':
            response.result=self._rieglVz.executeCommand(command)
        else:
            response.success=False
            response.message='Scanner not found'
        return response

    def _downloadCallback(self, request, response):
        self.get_logger().info("Service Request: download")
        save_location=request.new_file_location
        file_loaction=request.old_file_location
        response.success=True
        response.message='ok'
        status = self._rieglVz.getScannerStatus()
        if status.opstate != 'unavailable':
            response.message=self._rieglVz.download(file_loaction,save_location)
            if response.message != 'ok':
                response.success=False
        else:
            response.success=False
            response.message='Scanner not found'
        return response

    def _uploadCallback(self, request, response):
        self.get_logger().info("Service Request: upload")
        save_location=request.new_file_location
        file_loaction=request.old_file_location
        response.success=True
        response.message='ok'
        status = self._rieglVz.getScannerStatus()
        if status.opstate != 'unavailable':
            response.message=self._rieglVz.upload(file_loaction,save_location)
            if response.message != 'ok':
                response.success=False
        else:
            response.success=False
            response.message='Scanner not found'
        return response
        return response

    def shutdown(self):
        self._shutdownReq = True
        self.stop()
        self._rieglVz.shutdown()
        self._stopNodeReq = True

    def _shutdownCallback(self, request, response):
        self.get_logger().info("Service Request: shutdown")
        try:
            self.shutdown()
            self._setResponseSuccess(response)
        except Exception as e:
            self._setResponseException(response, e)

        return response

def stop_node():
    if _rieglVzWrapper is not None:
        _rieglVzWrapper.stop()

    try:
        rclpy.shutdown()
    except Exception as e:
        pass

def main(args=None):
    rclpy.init(args=args)
    _rieglVzWrapper = RieglVzWrapper()
    try:
        while not _rieglVzWrapper._stopNodeReq:
            rclpy.spin_once(_rieglVzWrapper)
    except KeyboardInterrupt:
        pass
    finally:
        stop_node()

if __name__ == "__main__":
    try:
        signal.signal(signal.SIGINT, stop_node)
        signal.signal(signal.SIGTERM, stop_node)
        main()
    except Exception as e:
        stop_node()
