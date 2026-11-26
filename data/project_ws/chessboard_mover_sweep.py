#!/usr/bin/env python3
"""Chessboard mover: smooth approach + close-range sweeps + tilts.

1. Approaches the stereo cameras from far to near (slow, small steps)
2. At closest distance, sweeps left <-> right repeatedly
3. Throughout: applies tilts for orientation diversity
"""

from __future__ import annotations

import math
import sys
import time

import numpy as np

for _p in ("/opt/ros/jazzy/opt/gz_transport_vendor/lib/python",
           "/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import gz.transport as t  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402

MODEL_NAME = "calibration_board"
POSE_PERIOD = 2.0        # seconds per pose (2s = stable frames at 30 Hz)

# Base orientation: board faces camera at origin.
# local +z (normal) -> world -x, local +y (up) -> world +z, local +x -> world -y.
BASE_R = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def _rot_xyz(r: float, p: float, y: float) -> np.ndarray:
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _rpy_of(R: np.ndarray):
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


def _rpy_to_quat(r: float, p: float, y: float):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    x = sr * cp * cy - cr * sp * sy
    yv = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, yv, z, w


def make_poses() -> list:
    """Generate pose sequence: approach -> close sweeps with tilts."""
    poses = []

    # 1. APPROACH PHASE: from far to near
    # Depth (x): start far, step toward camera. Camera at origin, board faces -x.
    # SDF initial pose: x=0.6. Keep z such that board clears ground (half-height 0.18 -> center z >= 0.18).
    approach_depths = [1.20, 1.00, 0.85, 0.70, 0.55, 0.45, 0.35, 0.30]
    approach_lateral = 0.0          # centered
    approach_heights = [0.35, 0.40, 0.45, 0.50]
    approach_tilts = [
        (0.0, 0.0, 0.0),
        (0.15, 0.10, 0.0), (-0.15, -0.10, 0.0),
        (0.0, 0.20, 0.0), (0.0, -0.20, 0.0),
    ]

    for i, x in enumerate(approach_depths):
        z = approach_heights[i % len(approach_heights)]
        for tr, tp, ty in approach_tilts:
            pos = np.array([x, approach_lateral, z])
            rpy = _rpy_of(BASE_R @ _rot_xyz(tr, tp, ty))
            poses.append((pos, rpy))

    # 2. CLOSE-RANGE SWEEPS: at closest depth, sweep left <-> right
    close_depth = 0.30   # as close as practical (board half-width ~0.24, stereo baseline ~0.12)
    close_heights = [0.25, 0.35, 0.45, 0.55]
    sweep_laterals = [-0.45, -0.30, -0.15, 0.0, 0.15, 0.30, 0.45]  # left -> center -> right
    close_tilts = [
        (0.0, 0.0, 0.0),
        (0.35, 0.25, 0.15), (-0.35, -0.25, -0.15),
        (0.50, -0.30, 0.20), (-0.50, 0.30, -0.20),
        (0.20, 0.50, -0.30), (-0.20, -0.50, 0.30),
        (0.60, 0.0, 0.10), (-0.60, 0.0, -0.10),
    ]

    # Multiple back-and-forth passes
    num_passes = 4
    for _ in range(num_passes):
        # Left to right
        for y in sweep_laterals:
            for z in close_heights:
                for tr, tp, ty in close_tilts:
                    pos = np.array([close_depth, y, z])
                    rpy = _rpy_of(BASE_R @ _rot_xyz(tr, tp, ty))
                    poses.append((pos, rpy))
        # Right to left (reverse)
        for y in reversed(sweep_laterals):
            for z in close_heights:
                for tr, tp, ty in close_tilts:
                    pos = np.array([close_depth, y, z])
                    rpy = _rpy_of(BASE_R @ _rot_xyz(tr, tp, ty))
                    poses.append((pos, rpy))

    return poses


def find_pose_service(node) -> str | None:
    for s in node.service_list():
        if s.startswith("/world/") and s.endswith("/set_pose"):
            return s
    return None


def main() -> None:
    node = t.Node()
    poses = make_poses()
    print(f"chessboard_mover_sweep: {len(poses)} poses, period={POSE_PERIOD}s, "
          f"repeating indefinitely", flush=True)

    svc = None
    for _ in range(60):
        svc = find_pose_service(node)
        if svc:
            break
        time.sleep(1)
    if not svc:
        print("chessboard_mover_sweep: could not find /world/*/set_pose service",
              flush=True)
        return
    print(f"chessboard_mover_sweep: using {svc}", flush=True)

    idx = 0
    n = len(poses)
    while True:
        pos, (r, p, y) = poses[idx % n]
        req = Pose()
        req.name = MODEL_NAME
        req.position.x = float(pos[0])
        req.position.y = float(pos[1])
        req.position.z = float(pos[2])
        qx, qy, qz, qw = _rpy_to_quat(r, p, y)
        req.orientation.x = qx
        req.orientation.y = qy
        req.orientation.z = qz
        req.orientation.w = qw

        ok, _resp = node.request(svc, req, Pose, Boolean, 2000)
        if ok:
            print(f"  [{idx + 1}] pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
                  f"rpy=({r:.2f},{p:.2f},{y:.2f}) -> ok", flush=True)
        else:
            print(f"  [{idx + 1}] -> service returned false", flush=True)
        idx += 1
        time.sleep(POSE_PERIOD)


if __name__ == "__main__":
    main()