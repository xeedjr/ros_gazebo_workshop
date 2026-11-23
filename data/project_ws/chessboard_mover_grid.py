#!/usr/bin/env python3
"""Wide, well-spread chessboard pose grid for stereo calibration.

Generates a grid of (depth, lateral, height, tilt) poses that spreads the
calibrator's 4-D parameter space (X, Y, Size, Skew) much more than the base
chessboard_mover (which uses 80 poses and plateaus at ~36 good samples).
Each grid axis is spaced so that distinct poses are L1-separated in parameter
space by > 0.2, letting the StereoCalibrator accept >= 40 good samples.

Run directly (no install needed):
    python3 chessboard_mover_grid.py
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
POSE_PERIOD = 2.0     # seconds held per pose (board is teleported; 2 s is stable)
RUN_SECONDS = 1200.0  # safety cap; the caller stops early on READY
PASSES = 3            # how many times to cycle the pose list

# Base orientation: vertical plate, face toward the camera at the origin.
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
    # Physical grid.  Depth drives Size, lateral drives X, height drives Y,
    # tilt drives Skew.  Keep the vertical plate (half-height 0.18 m) clear of
    # the ground: lowest centre z is 0.25 -> bottom edge ~0.07 m above ground.
    xs = [0.50, 0.85]                       # depth (m)  -> Size (2 levels)
    ys = [-0.45, -0.15, 0.15, 0.45]         # lateral (m) -> X (4 levels)
    zs = [0.25, 0.45, 0.70]                 # height (m)  -> Y (3 levels)
    tilts = [
        (0.0, 0.0, 0.0),
        (0.55, 0.35, 0.35), (-0.55, -0.35, -0.35),
        (0.70, -0.50, 0.20), (-0.70, 0.50, -0.20),
        (0.30, 0.70, -0.55), (-0.30, -0.70, 0.55),
    ]                                       # Skew (6 levels)
    poses = []
    i = 0
    for z in zs:
        for y in ys:
            for x in xs:
                for (tr, tp, ty) in tilts:
                    rpy = _rpy_of(BASE_R @ _rot_xyz(tr, tp, ty))
                    poses.append((np.array([x, y, z]), rpy))
                    i += 1
    return poses


def find_pose_service(node) -> str | None:
    for s in node.service_list():
        if s.startswith("/world/") and s.endswith("/set_pose"):
            return s
    return None


def main() -> None:
    node = t.Node()
    poses = make_poses()
    print(f"chessboard_mover_grid: {len(poses)} poses/pass, "
          f"period={POSE_PERIOD}s, passes={PASSES}, run cap={RUN_SECONDS:.0f}s",
          flush=True)

    svc = None
    for _ in range(60):
        svc = find_pose_service(node)
        if svc:
            break
        time.sleep(1)
    if not svc:
        print("chessboard_mover_grid: could not find /world/*/set_pose service",
              flush=True)
        return
    print(f"chessboard_mover_grid: using {svc}", flush=True)

    start = time.time()
    n = len(poses)
    idx = 0
    while time.time() - start < RUN_SECONDS and idx < n * PASSES:
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
        print(f"  [{idx + 1}] pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
              f"rpy=({r:.2f},{p:.2f},{y:.2f}) -> {'ok' if ok else 'false'}",
              flush=True)
        idx += 1
        time.sleep(POSE_PERIOD)

    print("chessboard_mover_grid: done", flush=True)


if __name__ == "__main__":
    main()
