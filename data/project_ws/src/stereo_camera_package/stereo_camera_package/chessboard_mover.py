#!/usr/bin/env python3
"""Teleport the kinematic ``calibration_board`` through a sequence of poses.

This lets the stereo calibration node observe the chessboard from many
viewpoints (different positions and tilts) so the stereo calibration has
diverse 3-D poses.

It mirrors the gz-transport pattern used by the launch file's auto-play script:
discover the world's ``/world/<name>/set_pose`` service and issue
``gz.msgs.Pose`` requests (the ``name`` field targets the board), holding each
pose for a few seconds so the 30 Hz cameras capture clean frames.
"""

from __future__ import annotations

import sys
import time
import math

import numpy as np

# The gz transport / msgs python bindings live in the vendor opt dir.
for _p in ("/opt/ros/jazzy/opt/gz_transport_vendor/lib/python",
           "/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python"):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import gz.transport as t  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402

MODEL_NAME = "calibration_board"
POSE_PERIOD = 4.0        # seconds to hold each pose
RUN_SECONDS = 300.0      # total wall-clock time the mover keeps teleporting

# Base orientation: plane local +z (normal) -> world -x (toward robot at origin),
# local +y (board up) -> world +z, local +x (board width) -> world -y.
# Equivalent to rpy roll=1.5708, pitch=0, yaw=-1.5708.
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
    """A deterministic, varied set of (position, rpy) poses in front of the
    cameras. Positions sweep x (depth), y (left/right) and z (up/down); small
    tilts are applied about the board's own axes for orientation diversity."""
    xs = [0.45, 0.60, 0.75, 0.90]
    ys = [-0.25, -0.10, 0.00, 0.10, 0.25]
    # Keep the vertical plate (half-height 0.18 m) clear of the ground plane:
    # the lowest pose's bottom edge then stays at z >= 0.25 - 0.18 = 0.07 m,
    # so the board never touches / collides with the ground.
    zs = [0.25, 0.35, 0.45, 0.55]
    tilts = [
        (0.0, 0.0, 0.0),
        (-0.4, -0.3, -0.3), (0.4, 0.3, 0.3),
        (-0.4, 0.4, 0.2), (0.4, -0.4, -0.2),
        (0.0, 0.5, 0.0), (0.0, -0.5, 0.0),
        (0.5, 0.0, 0.2), (-0.5, 0.0, -0.2),
        (0.2, 0.2, 0.4), (-0.2, -0.2, -0.4),
        (0.3, -0.2, 0.3), (-0.3, 0.2, -0.3),
        (0.15, 0.35, -0.25), (-0.15, -0.35, 0.25),
        (0.25, -0.35, 0.35), (-0.25, 0.35, -0.35),
    ]
    poses = []
    i = 0
    # Vary depth (x) fastest so the earliest poses span the full range of
    # distances. Changing parallax with depth is what best constrains the
    # relative camera rotation/translation in stereo calibration; holding depth
    # fixed lets the solver trade a spurious camera rotation for translation.
    for y in ys:
        for z in zs:
            for x in xs:
                tr, tp, ty = tilts[i % len(tilts)]
                pos = np.array([x, y, z])
                rpy = _rpy_of(BASE_R @ _rot_xyz(tr, tp, ty))
                poses.append((pos, rpy))
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
    print(f"chessboard_mover: {len(poses)} poses, period={POSE_PERIOD}s, "
          f"run={RUN_SECONDS:.0f}s", flush=True)

    svc = None
    for _ in range(60):
        svc = find_pose_service(node)
        if svc:
            break
        time.sleep(1)
    if not svc:
        print("chessboard_mover: could not find /world/*/set_pose service",
              flush=True)
        return
    print(f"chessboard_mover: using {svc}", flush=True)

    start = time.time()
    idx = 0
    while time.time() - start < RUN_SECONDS:
        pos, (r, p, y) = poses[idx % len(poses)]
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
            print(f"  pose {idx + 1}/{len(poses)}  "
                  f"pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})  "
                  f"rpy=({r:.2f},{p:.2f},{y:.2f})  -> ok", flush=True)
        else:
            print(f"  pose {idx + 1}/{len(poses)} -> service returned false",
                  flush=True)
        idx += 1
        time.sleep(POSE_PERIOD)

    print("chessboard_mover: done (run window elapsed)", flush=True)


if __name__ == "__main__":
    main()
