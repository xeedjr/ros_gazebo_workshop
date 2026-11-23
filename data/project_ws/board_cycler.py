#!/usr/bin/env python3
"""Cycle the calibration_board through a few well-framed, moderately-tilted
poses (centered in front of the cameras, at camera height) so the live evidence
script can capture clean, diverse board poses. Uses gz-transport like
chessboard_mover, but with a curated short pose list.
"""
import sys
import time
import math

import numpy as np

for _p in ("/opt/ros/jazzy/opt/gz_transport_vendor/lib/python",
           "/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import gz.transport as t  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402

MODEL_NAME = "calibration_board"
HOLD = float(__import__("os").environ.get("CYCLE_HOLD", "3.0"))

# local +z(normal)->world -x, +y(up)->world +z, +x(width)->world -y
BASE_R = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])

# Well-framed poses: centered (y small), near camera height (z~0.284),
# moderate depth (x 0.5-0.7), small tilts for orientation diversity.
POSES = [
    (0.60, 0.00, 0.284, 0.00, 0.00, 0.00),
    (0.55, 0.00, 0.284, 0.15, 0.10, 0.10),
    (0.65, 0.00, 0.284, -0.15, -0.10, -0.10),
    (0.60, 0.00, 0.284, 0.10, -0.15, 0.05),
    (0.55, 0.08, 0.284, -0.10, 0.12, -0.08),
    (0.65, -0.08, 0.284, 0.08, -0.10, 0.12),
    (0.60, 0.00, 0.350, 0.00, 0.20, 0.00),
    (0.60, 0.00, 0.220, -0.20, 0.00, 0.15),
    (0.50, 0.00, 0.284, 0.20, 0.00, -0.15),
    (0.70, 0.00, 0.284, -0.12, 0.12, 0.10),
]


def _rot_xyz(r, p, y):
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _rpy_of(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy < 1e-6:
        pitch = math.atan2(-R[2, 0], sy)
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    else:
        pitch = math.atan2(-R[2, 0], sy)
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def _rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    x = sr * cp * cy - cr * sp * sy
    yv = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, yv, z, w


def main():
    node = t.Node()
    svc = None
    for _ in range(60):
        for s in node.service_list():
            if s.startswith("/world/") and s.endswith("/set_pose"):
                svc = s
                break
        if svc:
            break
        time.sleep(1)
    if not svc:
        print("cycler: no /world/*/set_pose service", flush=True)
        return
    print(f"cycler: {len(POSES)} poses, hold={HOLD}s, svc={svc}", flush=True)

    while True:
        for i, (x, y, z, tr, tp, ty) in enumerate(POSES):
            r, p, yy = _rpy_of(BASE_R @ _rot_xyz(tr, tp, ty))
            req = Pose()
            req.name = MODEL_NAME
            req.position.x, req.position.y, req.position.z = float(x), float(y), float(z)
            qx, qy, qz, qw = _rpy_to_quat(r, p, yy)
            req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = qx, qy, qz, qw
            ok, _ = node.request(svc, req, Pose, Boolean, 2000)
            print(f"  pose {i+1}/{len(POSES)} pos=({x:.2f},{y:.2f},{z:.2f}) -> {'ok' if ok else 'FAIL'}",
                  flush=True)
            time.sleep(HOLD)


if __name__ == "__main__":
    main()
