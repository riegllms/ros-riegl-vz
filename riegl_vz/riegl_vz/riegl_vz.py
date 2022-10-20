import sys
import os
import time
import json
import math
from datetime import datetime
import subprocess
import threading
import numpy as np
from os.path import join, dirname, basename, abspath

from std_msgs.msg import (
    Header
)
from sensor_msgs.msg import (
    PointCloud2,
    PointField,
    NavSatStatus,
    NavSatFix
)
from geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseStamped,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    TransformStamped
)
from nav_msgs.msg import (
    Path,
    Odometry
)
import std_msgs.msg as std_msgs
import builtin_interfaces.msg as builtin_msgs

from rclpy.node import Node

import riegl.rdb

from vzi_services.controlservice import ControlService
from vzi_services.interfaceservice import InterfaceService
from vzi_services.projectservice import ProjectService
from vzi_services.scannerservice import ScannerService
from vzi_services.geosysservice import GeoSysService

from riegl_vz_interfaces.msg import (
    Voxels
)
from .pose import (
    readVop,
    readPop,
    readAllSopv,
    readFastSopv,
    readTpl,
    getTransformFromPose,
    calcRelativePose,
    calcRelativeCovariances,
    eulerFromQuaternion, quaternionFromEuler
)
from .tf2_geometry_msgs import (
    do_transform_pose
)
from .project import RieglVzProject
from .status import RieglVzStatus
from .geosys import RieglVzGeoSys
from .ssh import RieglVzSSH
from .utils import (
    SubProcess,
    parseCSV
)

appDir = dirname(abspath(__file__))

class ScanPattern(object):
    def __init__(self):
        self.lineStart = 30.0
        self.lineStop = 130.0
        self.lineIncrement = 0.04
        self.frameStart = 0.0
        self.frameStop = 360.0
        self.frameIncrement = 0.04
        self.measProgram = 3

class PositionWithCovariance(object):
    def __init__(self, position, covariance):
        self.position = position
        self.covariance = covariance

class YawAngleWithCovariance(object):
    def __init__(self, angle, covariance):
        self.angle = angle
        self.covariance = covariance

class ImuPose(object):
    def __init__(self, scanpos=None, pose=None):
        self.scanpos = scanpos
        self.pose = pose

    def isValid(self):
        return (self.scanpos is not None and self.pose is not None)

class ImuRelativePose(object):
    def __init__(self):
        self._pose = None
        self._previous = ImuPose()
        self._threadLock = threading.Lock()

    def _lock(self):
        self._threadLock.acquire()

    def _unlock(self):
        self._threadLock.release()

    def update(self, pose):
        self._lock()
        self._pose = pose
        self._unlock()

    def previous(self):
        self._lock()
        pose = self._previous
        self._unlock()
        return pose

    def reset(self):
        self._lock()
        self._previous = ImuPose()
        self._unlock()

    def get(self, scanposName):
        self._lock()
        poseCurrent = ImuPose(scanposName, self._pose)
        posePrevious = self._previous
        self._previous = poseCurrent
        self._pose = None
        self._unlock()
        return poseCurrent, posePrevious

class RieglVz():
    def __init__(self, node):
        self._node = node
        self._logger = node.get_logger()
        self._hostname = node.hostname
        self._workingDir = node.workingDir
        self._connectionString = self._hostname + ':20000'
        self._path = Path()
        self._yawAngle = None
        self._position = None
        self._imuRelPose = ImuRelativePose()
        self._stopReq = False
        self._shutdownReq = False
        self._project: RieglVzProject = RieglVzProject(self._node)
        self._status: RieglVzStatus = RieglVzStatus(self._node)
        self.geosys: RieglVzGeoSys = RieglVzGeoSys(self._node)
        self._ssh: RieglVzSSH = RieglVzSSH(self._node)

        self.scanposition = None

        self.scanPublishFilter = node.scanPublishFilter
        self.scanPublishLOD = node.scanPublishLOD

        if not os.path.exists(self._workingDir):
            os.mkdir(self._workingDir)

    def _broadcastTfTransforms(self, ts: datetime.time):
        ok, pop = self.getPop()
        if ok:
            self._node.transformBroadcaster.sendTransform(getTransformFromPose(ts, 'riegl_vz_prcs', pop))
        ok, vop = self.getVop()
        if ok:
            self._node.transformBroadcaster.sendTransform(getTransformFromPose(ts, 'riegl_vz_vocs', vop))
            ok, sopv = self.getSopv()
            if ok:
                self._node.transformBroadcaster.sendTransform(getTransformFromPose(ts, 'riegl_vz_socs', sopv.pose))
            else:
                return False, None
        else:
            return False, None
        return True, sopv

    def getScannerStatus(self):
        return self._status.status.getScannerStatus()

    def getScannerOpstate(self):
        return self.getScannerStatus().opstate

    def isScannerAvailable(self):
        return (self.getScannerOpstate() != 'unavailable')

    def getMemoryStatus(self):
        return self._status.status.getMemoryStatus()

    def getGnssStatus(self):
        return self._status.status.getGnssStatus()

    def getErrorStatus(self):
        return self._status.status.getErrorStatus()

    def getCameraStatus(self):
        return self._status.status.getCameraStatus()

    def loadProject(self, projectName: str, storageMedia: int, scanRegisterAndPublish: bool):
        ok = self._project.loadProject(projectName, storageMedia)
        if ok and scanRegisterAndPublish:
            ts = self._node.get_clock().now()
            self._broadcastTfTransforms(ts)
        if ok:
            self._imuRelPose.reset()
        return ok;

    def createProject(self, projectName: str, storageMedia: int):
        if self._project.createProject(projectName, storageMedia):
            self._path = Path();
            self._imuRelPose.reset()
            return True
        return False

    def getCurrentScanpos(self, projectName: str, storageMedia: int):
        return self._project.getCurrentScanpos(projectName, storageMedia);

    def getNextScanpos(self, projectName: str, storageMedia: int):
        return self._project.getNextScanpos(projectName, storageMedia)

    #def _getTimeStampFromScanId(self, scanId: str):
    #    scanFileName: str = os.path.basename(scanId)
    #    dateTime = datetime.strptime(scanFileName, '%y%m%d_%H%M%S.rxp')
    #    #self._logger.debug("dateTime = {}".format(dateTime))
    #    return int(dateTime.strftime("%s"))

    def _setPositionEstimate(self, position=None, yawAngle=None):
        scanposPath = self._project.getActiveScanposPath(self.scanposition)
        remoteFile = scanposPath + '/final.pose'
        localFile = self._workingDir + '/final.pose'
        self._ssh.downloadFile(remoteFile, localFile)
        with open(localFile, 'r') as f:
            finalPose = json.load(f)
        if position is not None:
            finalPose['positionEstimate'] = {
                'coordinateSystem': position.position.header.frame_id,
                'coord1': position.position.point.x,
                'coord2': position.position.point.y,
                'coord3': position.position.point.z,
                'coord1_conf': position.position.covariance[0],
                'coord2_conf': position.position.covariance[1],
                'coord3_conf': position.position.covariance[2]
            }
        if yawAngle is not None:
            finalPose['yaw'] = yawAngle.angle
            finalPose['yaw_conf'] = yawAngle.covariance
            finalPose['yaw_trust_level_high'] = True
        with open(localFile, 'w') as f:
            json.dump(finalPose, f, indent=4)
            f.write('\n')
        self._ssh.uploadFile([localFile], scanposPath)

    def _prepareImuRelativePose(self):
        # This will create a dummy 'imu_relative.pose' file in the scan position directory.
        # The trajectory service will not overwrite this file, instead it will create a file
        # 'imu_relative01.pose' (if scanner has been moved) which will not be used then.
        scanposPath = self._project.getActiveScanposPath(self.scanposition)
        localFile = self._workingDir + '/imu_relative.pose'
        with open(localFile, 'w') as f:
            f.write('{}\n')
        self._ssh.uploadFile([localFile], scanposPath)

    def _setImuRelativePose(self, posePrevious, poseCurrent):
        # This will create the file 'imu_relative.pose' with date from the external imu
        # and mode set to 'imu_external'.
        scanposPath = self._project.getActiveScanposPath(self.scanposition)
        localFile = self._workingDir + '/imu_relative.pose'
        pos_x, pos_y, pos_z, pos_roll, pos_pitch, pos_yaw = calcRelativePose(posePrevious.pose.pose.pose, poseCurrent.pose.pose.pose)
        cov_x, cov_y, cov_z, cov_roll, cov_pitch, cov_yaw = calcRelativeCovariances(posePrevious.pose.pose.covariance, poseCurrent.pose.pose.covariance)
        imuRelative = {
            'mode': 'imu_external',
            'origin': posePrevious.scanpos,
            'x': pos_x,
            'y': pos_y,
            'z': pos_z,
            'roll': pos_roll,
            'pitch': pos_pitch,
            'yaw': pos_yaw,
            'accuracy': {
                'x': cov_x,
                'y': cov_y,
                'z': cov_z,
                'roll': cov_roll,
                'pitch': cov_pitch,
                'yaw': cov_yaw
            }
        }
        with open(localFile, 'w') as f:
            json.dump(imuRelative, f, indent=4)
            f.write('\n')
        self._ssh.uploadFile([localFile], scanposPath)

    def _getGnssFixMessage(self, status=None):
        if status is None:
            status = self.getGnssStatus()

        if not status.valid or not status.publish:
            return False, None

        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'riegl_vz_gnss'

        if status.fix:
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position in degrees.
        msg.latitude = status.latitude
        msg.longitude = status.longitude

        # Altitude in metres.
        msg.altitude = status.altitude if not math.isnan(status.altitude) else 0.0

        lat_std = status.horAcc if not math.isnan(status.horAcc) else 0.0
        lon_std = status.horAcc if not math.isnan(status.horAcc) else 0.0
        alt_std = status.verAcc if not math.isnan(status.verAcc) else 0.0
        msg.position_covariance[0] = lat_std**2
        msg.position_covariance[4] = lon_std**2
        msg.position_covariance[8] = alt_std**2
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        return True, msg

    def publishGnssFix(self, status=None):
        ok, msg = self._getGnssFixMessage(status)
        if ok:
            self._node.gnssFixPublisher.publish(msg)

    def setProjectControlPoints(coordSystem: str, csvFile: str):
        projectPath = self._project.getActiveProjectPath()
        remoteSrcCpsFile = csvFile
        localSrcCpsFile = self._workingDir + '/' + os.path.basename(csvFile)
        self._ssh.downloadFile(remoteSrcCpsFile, localSrcCpsFile)
        csvData = parseCSV(localSrcCpsFile)[1:]
        # parse points and write resulting csv file
        controlPoints = []
        if csvData:
            if len(csvData[0]) < 4:
                raise RuntimeError("Invalid control points definition. File must have at least four columns.")
            localDstCpsFile = self._workingDir + '/controlpoints.csv'
            with open(localDstCpsFile, 'w') as f:
                f.write("Name,CRS,Coord1,Coord2,Coord3\n")
                for item in csvData:
                    f.write("{},{},{},{},{}\n".format(item[0], coordSystem, item[1], item[2], item[3]))
            self._ssh.uploadFile([localDstCpsFile], projectPath)

    def getPointCloud(self, scanposition: str, pointcloud: PointCloud2, ts = None):
        self._logger.debug("Downloading rdbx file..")
        self._status.status.setActiveTask('download rdbx file')
        scanId = self._project.getScanId(scanposition)
        self._logger.debug("scan id = {}".format(scanId))
        if scanId == 'null':
            self._logger.error("Scan id is null!")
            return False, pointcloud

        scanposPath = self._project.getActiveScanposPath(scanposition)
        self._logger.debug("scanpos path = {}".format(scanposPath))
        scan = os.path.basename(scanId).replace('.rxp', '')[0:13]
        self._logger.debug("scan = {}".format(scan))
        remoteFile = scanposPath + '/scans/' + scan + '.rdbx'
        localFile = self._workingDir + '/scan.rdbx'
        self._ssh.downloadFile(remoteFile, localFile)

        self._logger.debug("Generate point cloud..")
        self._status.status.setActiveTask('generate point cloud data')
        with riegl.rdb.rdb_open(localFile) as rdb:
            rosDtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize

            numTotalPoints = 0
            numPoints = 0
            data = bytearray()
            scanPublishLOD = self.scanPublishLOD
            if self.scanPublishLOD < 0:
                scanPublishLOD = 0
            for points in rdb.select(
                self.scanPublishFilter,
                chunk_size=100000
                ):
                pointStep = 2 ** scanPublishLOD
                for point in points:
                    if not (numTotalPoints % pointStep):
                        data.extend(point['riegl.xyz'].astype(dtype).tobytes())
                        data.extend(point['riegl.reflectance'].astype(dtype).tobytes())
                        numPoints += 1
                    numTotalPoints += 1

            fields = [PointField(
                name = n, offset = i*itemsize, datatype = rosDtype, count = 1)
                for i, n in enumerate('xyzr')]

            if ts:
                stamp = ts.to_msg()
            else:
                stamp = builtin_msgs.Time(sec = 0, nanosec = 0)

            header = std_msgs.Header(frame_id = 'riegl_vz_socs', stamp = stamp)

            pointcloud = PointCloud2(
                header = header,
                height = 1,
                width = numPoints,
                is_dense = False,
                is_bigendian = False,
                fields = fields,
                point_step = (itemsize * 4),
                row_step = (itemsize * 4 * numPoints),
                data = data
            )
            #for point in rdb.points():
            #    self._logger.debug("{0}".format(point.riegl_xyz))
        self._status.status.setActiveTask('')
        self._logger.debug("Point cloud generated.")

        return True, pointcloud

    def getVoxels(self, voxels: Voxels, scanposition: str = '0', ts: bool = True):
        self._logger.debug("Downloading vxls file..")
        self._status.status.setActiveTask('download vxls file')
        remoteFile = ''
        localFile = ''
        projectPath = self._project.getActiveProjectPath()
        self._logger.debug("project path = {}".format(projectPath))
        if scanposition != '1000000':
            remoteFile = projectPath + '/Voxels1.VPP/' + self._project.getScanposName(scanposition) + '.vxls'
            localFile = self._workingDir + '/scan.vxls'
        else:
            remoteFile = projectPath + '/Voxels1.VPP/project.vxls'
            localFile = self._workingDir + '/project.vxls'
        self._ssh.downloadFile(remoteFile, localFile)

        self._logger.debug("Generate voxels..")
        self._status.status.setActiveTask('generate voxel data')
        with riegl.rdb.rdb_open(localFile) as rdb:
            voxelSize = objs = float(json.loads(rdb.meta_data['riegl.voxel_info'])['size'])
            numTotalPoints = 0
            data = bytearray()
            for points in rdb.select('', chunk_size=100000):
                for point in points:
                    data.extend(point['riegl.xyz'].astype(np.float64).tobytes())
                    data.extend(point['riegl.reflectance'].astype(np.float32).tobytes())
                    data.extend(point['riegl.point_count'].astype(np.uint32).tobytes())
                    data.extend(point['riegl.pca_axis_min'].astype(np.float32).tobytes())
                    data.extend(point['riegl.pca_axis_max'].astype(np.float32).tobytes())
                    data.extend(point['riegl.pca_extents'].astype(np.float32).tobytes())
                    data.extend(point['riegl.shape_id'].astype(np.uint8).tobytes())
                    numTotalPoints += 1

            fields = []
            fieldsize = 0
            fields.append(PointField(name='x', offset=fieldsize, datatype=PointField.FLOAT64, count=1))
            fieldsize += np.dtype(np.float64).itemsize
            fields.append(PointField(name='y', offset=fieldsize, datatype=PointField.FLOAT64, count=1))
            fieldsize += np.dtype(np.float64).itemsize
            fields.append(PointField(name='z', offset=fieldsize, datatype=PointField.FLOAT64, count=1))
            fieldsize += np.dtype(np.float64).itemsize
            fields.append(PointField(name='r', offset=fieldsize, datatype=PointField.FLOAT32, count=1))
            fieldsize += np.dtype(np.float32).itemsize
            fields.append(PointField(name='point_count', offset=fieldsize, datatype=PointField.UINT32, count=1))
            fieldsize += np.dtype(np.uint32).itemsize
            fields.append(PointField(name='pca_axis_min', offset=fieldsize, datatype=PointField.FLOAT32, count=3))
            fieldsize += np.dtype(np.float32).itemsize * 3
            fields.append(PointField(name='pca_axis_max', offset=fieldsize, datatype=PointField.FLOAT32, count=3))
            fieldsize += np.dtype(np.float32).itemsize * 3
            fields.append(PointField(name='pca_extents', offset=fieldsize, datatype=PointField.FLOAT32, count=3))
            fieldsize += np.dtype(np.float32).itemsize * 3
            fields.append(PointField(name='shape_id', offset=fieldsize, datatype=PointField.UINT8, count=1))
            fieldsize += np.dtype(np.uint8).itemsize

            if ts:
                stamp = self._node.get_clock().now().to_msg()
            else:
                stamp = builtin_msgs.Time(sec = 0, nanosec = 0)

            header = std_msgs.Header(frame_id = 'riegl_vz_vocs', stamp = stamp)

            voxels = Voxels(
                voxel_size = voxelSize,
                pointcloud = PointCloud2(
                    header = header,
                    height = 1,
                    width = numTotalPoints,
                    is_dense = False,
                    is_bigendian = False,
                    fields = fields,
                    point_step = fieldsize,
                    row_step = (fieldsize * numTotalPoints),
                    data = data
                )
            )

        self._status.status.setActiveTask('')
        self._logger.debug("Voxels generated.")

        return True, voxels

    def _scanThreadFunc(self):
        ts = None
        self._status.status.setOpstate('scanning', 'scan data acquisition')
        self._status.status.setProgress(0)

        self._logger.info("Starting data acquisition..")
        self._logger.info("project name = {}".format(self.projectName))
        scanposName = self._project.getScanposName(self.scanposition)
        self._logger.info("scanpos name = {0} ({1})".format(self.scanposition, scanposName))
        self._logger.info("storage media = {}".format(self.storageMedia))
        self._logger.info("scan pattern = {0}, {1}, {2}, {3}, {4}, {5}".format(
            self.scanPattern.lineStart,
            self.scanPattern.lineStop,
            self.scanPattern.lineIncrement,
            self.scanPattern.frameStart,
            self.scanPattern.frameStop,
            self.scanPattern.frameIncrement))
        self._logger.info("meas program = {}".format(self.scanPattern.measProgram))
        self._logger.info("scan publish = {}".format(self.scanPublish))
        self._logger.info("scan publish filter = '{}'".format(self.scanPublishFilter))
        self._logger.info("scan publish LOD = {}".format(self.scanPublishLOD))
        self._logger.info("voxel publish = {}".format(self.voxelPublish))
        self._logger.info("scan register = {}".format(self.scanRegister))
        self._logger.info("scan register mode = {}".format(self.scanRegistrationMode))
        self._logger.info("pose publish = {}".format(self.posePublish))
        self._logger.info("pose publish fast = {}".format(self.posePublishFast))
        if self.reflSearchSettings:
            self._logger.info("reflector search = {}".format(self.reflSearchSettings))
        self._logger.info("image capture = {}".format(self.captureImages))
        self._logger.info("image capture mode = {}".format(self.captureMode))
        self._logger.info("image capture overlap = {}".format(self.imageOverlap))

        # prepare project
        try:
            projSvc = ProjectService(self._connectionString)
            projSvc.setStorageMedia(self.storageMedia)
            projSvc.createProject(self.projectName)
            projSvc.loadProject(self.projectName)
            projSvc.createScanposition(scanposName)
            projSvc.selectScanposition(scanposName)
            posePrevious = self._imuRelPose.previous()
            if posePrevious.isValid():
                self._prepareImuRelativePose()
        except:
            self._logger.error("Project and scan position prepare failed!")

        scriptPath = join(appDir, 'acquire-data.py')
        cmd = [
            'python3', scriptPath,
            '--connectionstring', self._connectionString]
        if self.reflSearchSettings:
            rssFilePath = join(self._workingDir, 'reflsearchsettings.json')
            with open(rssFilePath, 'w') as f:
                json.dump(self.reflSearchSettings, f)
            cmd.append('--reflsearch')
            cmd.append(rssFilePath)
        if self.scanPattern:
            cmd.extend([
                '--line-start', str(self.scanPattern.lineStart),
                '--line-stop', str(self.scanPattern.lineStop),
                '--line-incr', str(self.scanPattern.lineIncrement),
                '--frame-start', str(self.scanPattern.frameStart),
                '--frame-stop', str(self.scanPattern.frameStop),
                '--frame-incr', str(self.scanPattern.frameIncrement),
                '--measprog', str(self.scanPattern.measProgram)
            ])
        captureImages = (self.captureImages != 0)
        if self.captureImages == 2:
            if not self.getCameraStatus().avail:
                captureImages = False
        if captureImages:
            cmd.extend([
                '--capture-images',
                '--capture-mode', str(self.captureMode),
                '--image-overlap', str(self.imageOverlap)
            ])
        self._logger.debug("CMD = {}".format(' '.join(cmd)))
        subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self._logger.debug("Subprocess 'acquire-data' started.")
        subprocFastPosRdbx = None
        if self.scanRegister and self.posePublishFast and int(self.scanposition) > 1:
            self._logger.info("Starting fast pose rdbx creation..")
            time.sleep(1.0)
            scriptPath = join(appDir, 'fastpos-rdbx.py')
            cmd = [
                'python3', scriptPath,
                '--connectionstring', self._connectionString,
                '--sshuser', self._node.sshUser,
                '--sshpwd', self._node.sshPwd
            ]
            self._logger.debug("CMD = {}".format(' '.join(cmd)))
            subprocFastPosRdbx = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
            self._logger.debug("Subprocess 'fastpos-rdbx' started.")
        subproc.waitFor(errorMessage='Data acquisition failed.', block=True)
        if self._stopReq:
            self._stopReq = False
            self._status.status.setOpstate('waiting')
            self._logger.info("Scan stopped")
            return
        self._logger.info("Data acquisition finished.")

        self._status.status.setOpstate('processing')

        if self._position is not None or self._yawAngle is not None:
            if self._position is not None:
                self._status.status.setActiveTask('set position estimate')
            if self._yawAngle is not None:
                self._status.status.setActiveTask('set yaw angle estimate')
            if self._position is not None and int(self.scanposition) == 1:
                self._logger.info("Set project position.")
                projSvc = ProjectService(self._connectionString)
                projSvc.setProjectLocation(self._position.position.header.frame_id, self._position.position.point.x, self._position.position.point.y, self._position.position.point.z)
            self._logger.info("Set scan position and/or yaw angle estimate..")
            try:
                self._setPositionEstimate(self._position, self._yawAngle)
                self._logger.info("Set position and/or yaw angle estimate finished.")
            except:
                self._logger.error("Set position and/or yaw angle estimate failed!")
            self._position = None

        poseCurrent, posePrevious = self._imuRelPose.get(self._project.getScanposName(self.scanposition))
        if poseCurrent.isValid():
            self._logger.info("Set relative imu pose (current available).")
            if posePrevious.isValid():
                self._logger.info("Set relative imu pose (previous available).")
                self._status.status.setActiveTask('set relative imu pose')
                try:
                    self._setImuRelativePose(posePrevious, poseCurrent)
                    self._logger.info("Set relative imu pose finished.")
                except:
                    self._logger.error("Set relative imu pose failed!")
            self._logger.info("Set relative imu pose (previous = current).")

        if self.scanRegister and self.posePublishFast:
            sopvN = sopvN, sopvLast, sopvLastButOne = self._getLastSopvs()
            if sopvN >= 1:
                try:
                    self._status.status.setActiveTask('estimate fast pose')
                    subprocFastPosRdbx.waitFor(errorMessage='Creation of rdbx for fast pose failed.', block=True)
                    if self._stopReq:
                        self._stopReq = False
                        self._status.status.setOpstate('waiting')
                        self._logger.info("Scan stopped")
                        return
                    self._logger.info("Rdbx for fast pose created.")

                    self._logger.info("Starting fast registration..")
                    scriptPath = join(appDir, 'register-scan-coarse.py')
                    cmd = [
                        'python3', scriptPath,
                        '--hostname', self._hostname,
                        '--sshuser', self._node.sshUser,
                        '--sshpwd', self._node.sshPwd,
                        '--project-path', self._project.getActiveProjectPath(),
                        '--scanposition', scanposName,
                        '--scanposition-prev', self._project.getScanposName(str(sopvLast.seq))
                    ]
                    if sopvN >= 2:
                        cmd.extend(['--scanposition-pre-prev', self._project.getScanposName(str(sopvLastButOne.seq))])
                    cmd.extend(['--rdbx', '/tmp/fastpos.rdbx'])
                    self._logger.debug("CMD = {}".format(' '.join(cmd)))
                    subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
                    self._logger.debug("Subprocess 'coarse-registration' started.")
                    subproc.waitFor(errorMessage='Fast registration failed.', block=True)
                    if self._stopReq:
                        self._stopReq = False
                        self._status.status.setOpstate('waiting')
                        self._logger.info("Scan stopped")
                        return
                    self._logger.info("Fast registration finished.")

                    self._logger.info("Downloading and publishing fast pose..")
                    self._status.status.setActiveTask('publish fast pose')
                    ok, fastSopv = self.getFastSopv(scanposName)
                    if ok:
                        if ts is None:
                            ts = self._node.get_clock().now()
                        # update sopv timestamp
                        fastSopv.header.stamp = ts.to_msg()
                        # publish fast pose
                        self._node.poseFastPublisher.publish(fastSopv)
                        self._logger.info("Fast pose published.")
                    else:
                        self._logger.error("Fast pose download failed!")
                except:
                    self._status.status.addTaskError("FAST_REG_FAILED")
                    self._logger.error("Fast scan registration failed, fast pose is not being published!")

        if self.scanPublish:
            self._logger.info("Converting RXP to RDBX..")
            self._status.status.setActiveTask('convert rxp to rdbx')
            scriptPath = join(appDir, 'create-rdbx.py')
            cmd = [
                'python3', scriptPath,
                '--connectionstring', self._connectionString,
                '--project', self.projectName,
                '--scanposition', scanposName]
            self._logger.debug("CMD = {}".format(' '.join(cmd)))
            subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
            self._logger.debug("Subprocess started.")
            subproc.waitFor('RXP to RDBX conversion failed.')
            if self._stopReq:
                self._stopReq = False
                self._status.status.setOpstate('waiting')
                self._logger.info("Scan stopped")
                return
            self._logger.info("RXP to RDBX conversion finished.")

        scanRegisterFailed = False
        if self.scanRegister:
            self._logger.info("Starting registration..")
            self._status.status.setActiveTask('scan position registration')
            scriptPath = os.path.join(appDir, 'register-scan.py')
            cmd = [
                'python3', scriptPath,
                '--connectionstring', self._connectionString,
                '--project', self.projectName,
                '--scanposition', scanposName,
                '--registrationmode', str(self.scanRegistrationMode)]
            if self.posePublish:
                cmd.append('--wait-until-finished')
            self._logger.debug("CMD = {}".format(' '.join(cmd)))
            subproc = SubProcess(subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
            try:
                subproc.waitFor(errorMessage='Registration failed.', block=True)
                if self._stopReq:
                    self._stopReq = False
                    self._status.status.setOpstate('waiting')
                    self._logger.info("Scan stopped")
                    return
                self._logger.info("Registration finished.")

                if self.posePublish:
                    self._logger.info("Downloading and publishing pose..")
                    self._status.status.setActiveTask('publish registered scan position')
                    if ts is None:
                        ts = self._node.get_clock().now()
                    ok, sopv = self._broadcastTfTransforms(ts)
                    if ok:
                        # update sopv timestamp
                        sopv.pose.header.stamp = ts.to_msg()
                        # publish pose
                        self._node.posePublisher.publish(sopv.pose)
                        # publish path
                        self._path.header = sopv.pose.header
                        self._path.poses.append(sopv.pose)
                        self._node.pathPublisher.publish(self._path)
                        # publish odometry
                        odom = Odometry(
                            header = Header(stamp = ts.to_msg(), frame_id = 'riegl_vz_vocs'),
                            child_frame_id = 'riegl_vz_socs',
                            pose = PoseWithCovariance(pose = sopv.pose.pose)
                        )
                        self._node.odomPublisher.publish(odom)
                        self._logger.info("Pose published.")
            except:
                scanRegisterFailed = True
                self._status.status.addTaskError("SCAN_REG_FAILED")
                self._logger.error("Scan registration failed, pose is not being published!")

        if self.scanPublish:
            self._logger.info("Downloading and publishing point cloud..")
            pointcloud: PointCloud2 = PointCloud2()
            if ts is None:
                ts = self._node.get_clock().now()
            ok, pointcloud = self.getPointCloud(self.scanposition, pointcloud, ts)
            if ok:
                self._status.status.setActiveTask('publish point cloud data')
                self._node.pointCloudPublisher.publish(pointcloud)
            self._logger.info("Point cloud published.")

        if self.voxelPublish and self.scanRegister and not scanRegisterFailed:
            self._logger.info("Downloading and publishing voxel data..")
            voxels: Voxels = Voxels()
            ok, voxels = self.getVoxels(voxels, self.scanposition)
            if ok:
                self._status.status.setActiveTask('publish voxel data')
                self._node.voxelsPublisher.publish(voxels)
            self._logger.info("Voxels published.")

        self._status.status.setOpstate('waiting')

    def scan(
        self,
        projectName: str,
        scanposition: str,
        storageMedia: int,
        scanPattern: ScanPattern,
        scanPublish: bool = True,
        scanPublishFilter: str = '',
        scanPublishLOD: int = 1,
        voxelPublish: bool = False,
        scanRegister: bool = True,
        scanRegistrationMode: int = 1,
        posePublish: bool = True,
        posePublishFast: bool = False,
        reflSearchSettings: dict = None,
        captureImages: int = 2,
        captureMode: int = 1,
        imageOverlap: int = 25):
        """Acquire data at scan position.

        Args:
          projectName ... the project name
          scanposition ... the name of the new scan position
          storageMedia ... storage media for data recording
          scanPattern ... the scan pattern
          reflSearchSettings ... reflector search settings"""

        if self.isBusy(block=False):
            return False

        self.projectName = projectName
        self.scanposition = scanposition
        self.storageMedia = storageMedia
        self.scanPattern = scanPattern
        self.scanPublish = scanPublish
        self.scanPublishFilter = scanPublishFilter
        self.scanPublishLOD = scanPublishLOD
        self.voxelPublish = voxelPublish
        self.scanRegister = scanRegister
        self.scanRegistrationMode = scanRegistrationMode
        self.posePublish = posePublish
        self.posePublishFast = posePublishFast
        self.reflSearchSettings = reflSearchSettings
        self.captureImages = captureImages
        self.captureMode = captureMode
        self.imageOverlap = imageOverlap

        self._status.status.clearTaskErrors()

        thread = threading.Thread(target=self._scanThreadFunc, args=())
        thread.daemon = True
        thread.start()

        while not self.isScanning(block=False):
            time.sleep(0.2)

        return True

    def isScanning(self, block = True):
        if block:
            while self.getScannerOpstate() == 'scanning':
                time.sleep(0.2)
        return True if self.getScannerOpstate() == 'scanning' else False

    def isBusy(self, block = True):
        if block:
            while self.getScannerOpstate() != 'waiting':
                time.sleep(0.2)
        return False if self.getScannerOpstate() == 'waiting' else True

    def setPosition(self, position, covariance):
        if position.header.frame_id != '' and position.header.frame_id != 'riegl_vz_prcs':
            try:
                # try to convert to PRCS
                position = self._node.transformBuffer.transform(position, 'riegl_vz_prcs')
            except:
                self._logger.warning("Position coordinate transformation to PRCS failed!")
        self._position = PositionWithCovariance(position, covariance)


    def _setYawAngle(self, header, yawAngle, covariance):
        if header.frame_id != '' and header.frame_id != 'riegl_vz_prcs':
            # must be converted to PRCS
            pose = PoseStamped()
            pose.header = header
            pose.pose = Pose(
                position = Point(x=0.0, y=0.0, z=0.0),
                orientation = quaternionFromEuler(0.0, 0.0, yawAngle)
            )
            pose = self._node.transformBuffer.transform(pose, 'riegl_vz_prcs')
            roll, pitch, yawAngle = eulerFromQuaternion(pose.pose.orientation)
        self._yawAngle = YawAngleWithCovariance(yawAngle, covariance)

    def setPose(self, pose, isRelative, mountingPose):
        self.robotRelativePose = isRelative
        self._logger.info("robot relative pose = {}".format(self.robotRelativePose))
        self._logger.info("robot scanner mounting pose = x: {0}, y: {1}, z: {2}, roll: {3}, pitch: {4}, yaw: {5}".format(mountingPose[0], mountingPose[1], mountingPose[2], mountingPose[3], mountingPose[4], mountingPose[5]))
        if self.robotRelativePose:
            try:
                # try to set yaw angle
                trans = TransformStamped()
                trans.transform.translation.x = mountingPose[0]
                trans.transform.translation.y = mountingPose[1]
                trans.transform.translation.z = mountingPose[2]
                self._logger.error("euler = {0} {1} {2}".format(mountingPose[0], mountingPose[1], mountingPose[2]))
                trans.transform.rotation = quaternionFromEuler(mountingPose[3], mountingPose[4], mountingPose[5])
                self._logger.error("trans = {}".format(trans))
                self._logger.error("pose = {}".format(pose.pose.pose))
                pose2 = do_transform_pose(pose.pose.pose, trans)
                self._logger.error("pose2 = {}".format(pose2))
                roll, pitch, yaw = eulerFromQuaternion(pose2.orientation)
                self._logger.error("euler2 = {}".format(eulerFromQuaternion(pose2.orientation)))
                cov = np.array(pose.pose.covariance).reshape(6,6)
                self._setYawAngle(pose.header, yaw, cov[5][5].item())
            except:
                self._logger.warning("Yaw angle configuration with transformation to PRCS failed!")
            self._imuRelPose.update(pose)
        else:
            trans = TransformStamped()
            trans.transform.translation.x = mountingPose[0]
            trans.transform.translation.y = mountingPose[1]
            trans.transform.translation.z = mountingPose[2]
            trans.transform.rotation = quaternionFromEuler(mountingPose[3], mountingPose[4], mountingPose[5])
            pose2 = do_transform_pose(pose.pose.pose, trans)
            roll, pitch, yaw = eulerFromQuaternion(pose2.orientation)
            cov = np.array(pose.pose.covariance).reshape(6,6)
            self._setYawAngle(pose.header, yaw, cov[5][5].item())
            position = PointStamped(
                header = pose.header,
                point = pose2.position
            )
            self.setPosition(position, [cov[0][0].item(), cov[1][1].item(), cov[2][2].item()])

    def getAllSopv(self):
        try:
            sopvFileName = 'all_sopv.csv'
            remoteFile = self._project.getActiveProjectPath() + '/Voxels1.VPP/' + sopvFileName
            localFile = self._workingDir + '/' + sopvFileName
            self._ssh.downloadFile(remoteFile, localFile)
            ok = True
            sopvs = readAllSopv(localFile, self._logger)
        except Exception as e:
            ok = False
            sopvs = None

        return ok, sopvs

    def getFastSopv(self, scanposition: str):
        try:
            sopvFileName = 'robot_fast_pose.sopv'
            remoteFile = self._project.getActiveProjectPath() + '/' + scanposition + '.SCNPOS/robot_fast_pose.sopv'
            localFile = self._workingDir + '/' + sopvFileName
            self._ssh.downloadFile(remoteFile, localFile)
            ok = True
            sopv = readFastSopv(localFile, self._logger)
        except Exception as e:
            ok = False
            sopv = None

        return ok, sopv

    def getSopv(self):
        ok, sopvs = self.getAllSopv()
        if ok and len(sopvs):
            sopv = sopvs[-1]
        else:
            sopv = None
            ok = False

        return ok, sopv

    def _getLastSopvs(self):
        n = 0
        sopvLast = -1
        sopvLastButOne = -1
        ok, sopvs = self.getAllSopv()
        if ok:
            n = len(sopvs)
            if n > 0:
                sopvLast = sopvs[-1]
            if n > 1:
                sopvLastButOne = sopvs[-2]
        return n, sopvLast, sopvLastButOne

    def getVop(self):
        try:
            sopvFileName = 'VPP.vop'
            remoteFile = self._project.getActiveProjectPath() + '/Voxels1.VPP/' + sopvFileName
            localFile = self._workingDir + '/' + sopvFileName
            self._ssh.downloadFile(remoteFile, localFile)
        except Exception as e:
            return False, None

        vop = readVop(localFile)

        return True, vop

    def getPop(self):
        try:
            popFileName = 'project.pop'
            remoteFile = self._project.getActiveProjectPath() + '/' + popFileName
            localFile = self._workingDir + '/' + popFileName
            self._ssh.downloadFile(remoteFile, localFile)
        except Exception as e:
            return False, None

        pop = readPop(localFile)

        return True, pop

    def getTpl(self, scanposition: str):
        try:
            scanId = self._project.getScanId(scanposition)
            self._logger.debug("scan id = {}".format(scanId))
            if scanId == 'null':
                self._logger.error("Scan id is null!")
                return False, None

            scanposPath = self._project.getActiveScanposPath(scanposition)
            self._logger.debug("scanpos path = {}".format(scanposPath))
            scan = os.path.basename(scanId).replace('.rxp', '')[0:13]
            self._logger.debug("scan = {}".format(scan))
            remoteFile = scanposPath + '/' + scan + '.tpl'
            localFile = self._workingDir + '/scan.tpl'
            self._ssh.downloadFile(remoteFile, localFile)

            ok = True
            tpl = readTpl(localFile, self._logger)
        except Exception as e:
            ok = False
            tpl = None

        return ok, tpl

    def stop(self):
        self._stopReq = True

        if self.isScannerAvailable():
            ctrlSvc = ControlService(self._connectionString)
            ctrlSvc.stop()
            self.isBusy()

    def trigStartStop(self):
        trigStartedPrev = self._status.trigStarted
        if not self._status.trigStarted:
            if self.isBusy(block = False):
                return False
            self._status.trigStarted = True

        intfSvc = InterfaceService(self._connectionString)
        intfSvc.triggerInputEvent('ACQ_START_STOP')

        if not trigStartedPrev and self._status.trigStarted:
            startTime = time.time()
            while not self.isScanning(block=False):
                time.sleep(0.2)
                if (time.time() - startTime) > 5:
                    self._status.trigStarted = False
                    return False

        return True

    def getScanPatterns(self):
        patterns: str = []
        ctrlSvc = ControlService(self._connectionString)
        for pattern in json.loads(ctrlSvc.scanPatternsDetailed()):
            patterns.append(pattern['name'])
        #instIdentLower = self._status.status.scannerStatus.instIdent.lower()
        #remotePath = '/usr/share/gui/' + instIdentLower + '/patterns'
        #files = self._ssh.listFiles(remotePath, '*.pat')
        #for file in files:
        #    patterns.append(os.path.basename(file).replace('.pat', ''))
        return True, patterns

    def getScanPattern(self, patternName):
        ctrlSvc = ControlService(self._connectionString)
        for p in json.loads(ctrlSvc.scanPatternsDetailed()):
            if p['name'] == patternName:
                pattern: ScanPattern = ScanPattern()
                pattern.lineStart = p['thetaStart']
                pattern.lineStop = p['thetaStop']
                pattern.lineIncrement = p['thetaIncrement']
                pattern.frameStart = p['phiStart']
                pattern.frameStop = p['phiStop']
                pattern.frameIncrement = p['phiIncrement']
                return True, pattern
        self._logger.error("Scan pattern '{}' is not available!")
        return False, None

    def getReflectorModels(self):
        models: str = []
        ctrlSvc = ControlService(self._connectionString)
        for model in json.loads(ctrlSvc.supportedReflectorSearchModels()):
            models.append(model['name'])
        return True, models

    def transformGeoCoordinate(self, srcCs: str, dstCs: str, coord1=0, coord2=0, coord3=0):
        return self.geosys.transformCoordinate(srcCs, dstCs, coord1, coord2, coord3)

    def shutdown(self):
        self._status.shutdown()
        self.stop()
        if self.isScannerAvailable():
            scnSvc = ScannerService(self._connectionString)
            scnSvc.shutdown()
