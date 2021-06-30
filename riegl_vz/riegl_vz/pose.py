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
    PoseStamped
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
    with open(vopPath, "r") as f:
        vpp_vop = json.load(f)

    vop = PoseStamped()
    vop.header = Header(
        frame_id = "RIEGL_PRCS",
        stamp = builtin_msgs.Time(sec = 0, nanosec = 0)
        )
    vop.pose = Pose(
        position = Point(x=vpp_vop["translation"]["x"], y=vpp_vop["translation"]["y"], z=vpp_vop["translation"]["z"]),
        orientation = quaternionFromRotationMatrix(vpp_vop["matrix3x3"])
        )

    return vop

def extractSopv(data):
    """Extract scanposition information from all_sopv.csv data entry."""
    deg = math.pi / 180.0
    obj = {
        "name": data[0],
        "x": float(data[1]),
        "y": float(data[2]),
        "z": float(data[3]),
        "roll": float(data[4])*deg,
        "pitch": float(data[5])*deg,
        "yaw": float(data[6])*deg
    }
    pose = PoseStamped()
    pose.header = Header(
        frame_id = "RIEGL_VOCS",
        stamp = builtin_msgs.Time(sec = 0, nanosec = 0)
        )
    pose.pose = Pose(
        position = Point(x=obj["x"], y=obj["y"], z=obj["z"]),
        orientation = quaternionFromEuler(obj["roll"], obj["pitch"], obj["yaw"])
        )

    sopv = ScanPose(seq = int(obj["name"]), pose = pose)

    return sopv

def readAllSopv(sopvFilepath):
    """Return information of all registered scanposes in VOCS."""
    sopvs = []
    first_line = True
    with open(sopvFilepath, "r") as f:
        for line in f:
            if first_line:
                first_line = False
                continue
            sopvs.append(extractSopv(line.split(",")))
    return sopvs
