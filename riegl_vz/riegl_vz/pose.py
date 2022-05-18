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

def quaternionToRotationMatrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q.w
    q1 = Q.x
    q2 = Q.y
    q3 = Q.z

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    R = np.array([[r00, r01, r02],
                  [r10, r11, r12],
                  [r20, r21, r22]])

    return R

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

def eulerFromQuaternion(Q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (Q.w * Q.x + Q.y * Q.z)
    t1 = +1.0 - 2.0 * (Q.x * Q.x + Q.y * Q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (Q.w * Q.y - Q.z * Q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (Q.w * Q.z + Q.x * Q.y)
    t4 = +1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def eulerFromRotationMatrix(R):
    """Calculates rotation matrix to euler angles.
    Return list of (roll, pitch, yaw) in radians."""
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return x, y, z # in radians

def rotationMatrixFromEuler(roll, pitch, yaw):
    """Calculates Rotation Matrix given euler angles.
    Args:
      roll  ... X rotation angle in radians,
      pitch ... Y rotation angle in radians,
      yaw   ... Z rotation angle in radians"""
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]
                    ])

    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]
                    ])

    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]
                    ])

    return R_z @ R_y @ R_x

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
        frame_id = glcs,
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

def getTransformFromArray(ts, parent_frame_id, child_frame_id, pose):
    """Return tf2 TransformStamped message from pose."""
    t = TransformStamped()
    t.header.frame_id = parent_frame_id
    t.header.stamp = ts.to_msg()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    t.transform.rotation = quaternionFromEuler(pose[3], pose[4], pose[5])
    return t

def calcRelativePose(pose1, pose2):
    # Wenn wir die 4x4-Matrizen (S1, S2) hätten, würde man die relative Pose wie folgt erhalten:
    # M_rel = (S1)^(-1) * S2
    #
    # Zerlegt man die 4x4-Matrizen in die Drehmatrizen (R1, R2) und die Positionen (P1, P2), gilt für die Transformation eines Punktes vom lokalen ins globale System:
    # X = R1 * x1 + P1
    # X = R2 * x2 + P2
    #
    # x1, x2, und X sind ein und derselbe Punkt, jedoch sind x1 dessen Koordinaten in System 1, x2 jene in System 2 und X im übergeordneten System.
    #
    # Wir brauchen die Transformation von x2 nach x1:
    # Die erste Gleichung nach x1 umgeformt ergibt:
    # x1 = R1^(-1) * (X – P1)
    # Durch Einsetzen der zweiten Gleichung erhält man:
    # x1 = R1^(-1) * (R2 * x2 + P2 – P1)
    # Das lässt sich durch Ausmultiplizieren in Drehung und Verschiebung trennen:
    # x1 = R1^(-1) * R2 * x2 + R1^(-1) * (P2 – P1) = R_rel * x2 + P_rel
    # Relative Drehung: R_rel = R1^(-1) * R2 = R1T * R2
    # Relative Verschiebung: P_rel = R1^(-1) * (P2 – P1) = R1T * (P2 – P1)
    q1 = Quaternion()
    q1.x = pose1.orientation.x
    q1.y = pose1.orientation.y
    q1.z = pose1.orientation.z
    q1.w = pose1.orientation.w
    R1 = quaternionToRotationMatrix(q1)
    q2 = Quaternion()
    q2.x = pose2.orientation.x
    q2.y = pose2.orientation.y
    q2.z = pose2.orientation.z
    q2.w = pose2.orientation.w
    R2 = quaternionToRotationMatrix(q2)
    R_rel = np.dot(np.linalg.inv(R1), R2)
    roll, pitch, yaw = eulerFromRotationMatrix(R_rel)

    P1 = np.array([[pose1.position.x], [pose1.position.y], [pose1.position.z]])
    P2 = np.array([[pose2.position.x], [pose2.position.y], [pose2.position.z]])
    P_rel = np.dot(np.linalg.inv(R1), np.subtract(P2,P1))
    x = P_rel[0].item()
    y = P_rel[1].item()
    z = P_rel[2].item()

    return x, y, z, roll, pitch, yaw

def calcRelativeCovariances(pose1, pose2):
    # @@AF: FIXME
    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0
    return x, y, z, roll, pitch, yaw
