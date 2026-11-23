#!/usr/bin/env python3
"""Re-run stereo calibration from the saved images and write a CORRECTED
calibration tarball (proper projection matrices).

Mirrors the headless node: same findChessboardCornersSB corner patch, and the
post-calibration stereoRectify recompute with fresh output buffers (workaround
for the OpenCV 4.6 in-place stereoRectify P corruption).
"""
import sys
import tarfile

import cv2
import numpy as np

sys.path.insert(0, "/opt/ros/jazzy/lib/python3.12/site-packages")
import camera_calibration.calibrator as _cal_mod
from camera_calibration.calibrator import (StereoCalibrator, ChessboardInfo,
                                           Patterns)


def _sb_get_corners(img, board, refine=True, checkerboard_flags=0):
    h, w = img.shape[:2]
    mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    ok, corners = cv2.findChessboardCornersSB(mono, (board.n_cols, board.n_rows))
    if not ok:
        return (False, None)
    border = 8
    for i in range(corners.shape[0]):
        if not (border < corners[i, 0, 0] < w - border
                and border < corners[i, 0, 1] < h - border):
            return (False, None)
    if board.n_rows != board.n_cols:
        if corners[0, 0, 1] > corners[-1, 0, 1]:
            corners = np.copy(np.flipud(corners))
    return (True, corners)


_cal_mod._get_corners = _sb_get_corners


def recompute_stereo_rectify(calib, log):
    c = calib
    res = cv2.stereoRectify(c.l.intrinsics, c.l.distortion,
                            c.r.intrinsics, c.r.distortion,
                            c.size, c.R, c.T, None, None, None, None)
    c.l.R[:] = res[0]
    c.r.R[:] = res[1]
    c.l.P[:] = res[2]
    c.r.P[:] = res[3]
    try:
        cv2.initUndistortRectifyMap(c.l.intrinsics, c.l.distortion, c.l.R,
                                    c.l.P, c.size, cv2.CV_32FC1,
                                    c.l.mapx, c.l.mapy)
        cv2.initUndistortRectifyMap(c.r.intrinsics, c.r.distortion, c.r.R,
                                    c.r.P, c.size, cv2.CV_32FC1,
                                    c.r.mapx, c.r.mapy)
    except Exception as exc:
        log(f"recompute rectify maps: {exc}")


SRC = "/tmp/calibrationdata.tar.gz"
DST = "/tmp/calibrationdata_fixed.tar.gz"

src = tarfile.open(SRC, "r:gz")
names = sorted(src.getnames())
limages = [cv2.imdecode(np.frombuffer(src.extractfile(n).read(), np.uint8),
                        cv2.IMREAD_COLOR)
           for n in names if n.startswith("left-") and n.endswith(".png")]
rimages = [cv2.imdecode(np.frombuffer(src.extractfile(n).read(), np.uint8),
                        cv2.IMREAD_COLOR)
           for n in names if n.startswith("right-") and n.endswith(".png")]
print(f"loaded {len(limages)} left / {len(rimages)} right images")

cal = StereoCalibrator([ChessboardInfo("chessboard", 9, 6, 0.04)], 0, 0,
                       Patterns.Chessboard, name="stereo_camera")
cal.cal(limages, rimages)
print("in-cal left P fx =", cal.l.P[0, 0])
recompute_stereo_rectify(cal, print)
print("fixed  left P fx =", cal.l.P[0, 0])

dst = tarfile.open(DST, "w:gz")
cal.do_tarfile_save(dst)
dst.close()

print("cv2:", cv2.__version__)
print("self.T =", np.ravel(cal.T).tolist(), " |T| =", round(np.linalg.norm(cal.T), 5))
print("left  P =\n", cal.l.P)
print("right P =\n", cal.r.P)
print("baseline from P (m):", round(abs(cal.r.P[0, 3]) / cal.r.P[0, 0], 5))
print("wrote", DST)
