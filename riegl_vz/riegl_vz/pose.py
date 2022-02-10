import math
import json
import numpy as np

import builtin_interfaces.msg as builtin_msgs
from std_msgs.msg import (
    Header
)
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
    PoseStamped,
    TransformStamped
)
from riegl_vz_interfaces.msg import (
    ScanPose
)

def quaternionFromRotationMatrix(R):
    """
    Calculates rotation matrix to quaternion.
    """
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0

    return quaternionFromEuler(roll, pitch, yaw)

def quaternionFromEuler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

def readVop(vopPath):
    """Extract VOP pose."""
    with open(vopPath, 'r') as f:
        vpp_vop = json.load(f)

    x = float(vpp_vop['translation']['x'])
    y = float(vpp_vop['translation']['y'])
    z = float(vpp_vop['translation']['z'])

    R = np.empty((3,3))
    R[:3, :3] = vpp_vop['matrix3x3']

    vop = PoseStamped()
    vop.header = Header(
        frame_id = 'riegl_vz_prcs',
        stamp = builtin_msgs.Time(sec = 0, nanosec = 0)
        )
    vop.pose = Pose(
        position = Point(x=x, y=y, z=z),
        orientation = quaternionFromRotationMatrix(R)
        )

    return vop

def readPop(popPath):
    """Extract POP pose."""
    with open(popPath, 'r') as f:
        obj = json.load(f)
        pop = np.array(obj.get('4x4'))
        glcs = 'EPSG::4978'
        if 'glcs' in obj:
            glcs = obj['glcs'].get('designator', glcs)

    T = np.empty(3)
    T = pop[:3, 3]

    R = np.empty((3,3))
    R[:3, :3] = pop[:3, :3]

    pop = PoseStamped()
    pop.header = Header(
        frame_id = 'riegl_vz_glcs',
        stamp = builtin_msgs.Time(sec = 0, nanosec = 0)
        )
    pop.pose = Pose(
        position = Point(x=T[0], y=T[1], z=T[2]),
        orientation = quaternionFromRotationMatrix(R)
        )

    return pop

def extractSopv(data, logger = None):
    """Extract scanposition information from all_sopv.csv data entry."""
    deg = math.pi / 180.0
    obj = {
        'name': data[0],
        'x': float(data[1]),
        'y': float(data[2]),
        'z': float(data[3]),
        'roll': float(data[4])*deg,
        'pitch': float(data[5])*deg,
        'yaw': float(data[6])*deg
    }
    if logger is not None:
        logger.debug("sopv (x y z yaw pitch roll)= {0} {1} {2} {3} {4} {5}".format(
            obj['x'],
            obj['y'],
            obj['z'],
            obj['yaw'],
            obj['pitch'],
            obj['roll']))
    pose = PoseStamped()
    pose.header = Header(
        frame_id = 'riegl_vz_vocs',
        stamp = builtin_msgs.Time(sec = 0, nanosec = 0)
        )
    pose.pose = Pose(
        position = Point(x=float(obj['x']), y=float(obj['y']), z=float(obj['z'])),
        orientation = quaternionFromEuler(float(obj['roll']), float(obj['pitch']), float(obj['yaw']))
        )

    sopv = ScanPose(seq = int(obj['name']), pose = pose)

    return sopv

def readAllSopv(sopvFilepath, logger = None):
    """Return information of all registered scanposes in VOCS."""
    sopvs = []
    first_line = True
    with open(sopvFilepath, 'r') as f:
        for line in f:
            if first_line:
                first_line = False
                continue
            sopvs.append(extractSopv(line.split(','), logger))
    return sopvs

def getTransformFromPose(ts, child_frame_id, pose):
    """Return tf2 TransformStamped message from pose."""
    t = TransformStamped()
    t.header = pose.header
    t.header.stamp = ts.to_msg()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = pose.pose.position.x
    t.transform.translation.y = pose.pose.position.y
    t.transform.translation.z = pose.pose.position.z
    t.transform.rotation = pose.pose.orientation
    return t
