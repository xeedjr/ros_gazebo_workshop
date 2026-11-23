#!/usr/bin/env python3
"""Live verification on the running sim.

Subscribes to the sim stereo stream, APPLIES the recalibrated yaml (R1/P1,
R2/P2), and measures, over many board poses:

  (A) rectified epipolar alignment: the same chessboard corner must land on the
      same image row in both rectified views (sub-pixel => disparity is valid);
  (B) SGBM disparity quality: valid-pixel fraction, mean/median disparity, and
      a saved disparity map;
  (C) triangulated chessboard geometry: recovered square size (true 0.04 m) and
      coplanarity residual => the disparity encodes a correct 3-D grid.

Run inside the container while the sim is up:
  source /opt/ros/jazzy/setup.bash && source /root/data/project_ws/install/setup.sh
  python3 /root/data/project_ws/evidence_live.py
"""
import os
import json
import time

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

CALIB_DIR = os.environ.get("CALIB_DIR", "/root/data/project_ws/calibration_sim")
OUT = os.environ.get("EVIDENCE_OUT", "/root/data/project_ws/calibration_sim/evidence")
N_FRAMES = int(os.environ.get("N_FRAMES", "24"))
CENTROID_JUMP = 10.0
MAX_WAIT_S = float(os.environ.get("MAX_WAIT_S", "200"))
COLS, ROWS = 9, 6
TRUE_SQUARE = 0.04
os.makedirs(OUT, exist_ok=True)


def block(doc, key):
    b = doc[key]
    return np.array(b["data"], dtype=float).reshape(b["rows"], b["cols"])


def load(side):
    with open(os.path.join(CALIB_DIR, f"{side}_sim.yaml")) as f:
        d = yaml.safe_load(f)
    return dict(
        K=block(d, "camera_matrix"),
        D=np.array(d["distortion_coefficients"]["data"], dtype=float),
        R=block(d, "rectification_matrix"),
        P=block(d, "projection_matrix"),
        w=int(d["image_width"]), h=int(d["image_height"]),
    )


L = load("left")
R = load("right")
w, h = L["w"], L["h"]

# Precompute undistort+rectify maps for both cameras.
mapLx, mapLy = cv2.initUndistortRectifyMap(L["K"], L["D"], L["R"], L["P"], (w, h), cv2.CV_16SC2)
mapRx, mapRy = cv2.initUndistortRectifyMap(R["K"], R["D"], R["R"], R["P"], (w, h), cv2.CV_16SC2)

# SGBM (3-way) for the disparity map.
sbm = cv2.StereoSGBM_create(
    minDisparity=0, numDisparities=96, blockSize=9,
    uniquenessRatio=10, speckleWindowSize=100, speckleRange=32,
    disp12MaxDiff=1, preFilterCap=63, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)


def find(gray):
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, (COLS, ROWS))
        return bool(ok), corners
    ok, corners = cv2.findChessboardCorners(gray, (COLS, ROWS))
    return bool(ok), corners


def rect_pix(corners, C):
    """Distorted corners (N,1,2) -> rectified pixel coords (N,2).

    x = undistortPoints(corner, K, D) is K^-1 * ideal, the normalized point in
    the ORIGINAL camera frame (z=1). The rectified projection matrix P already
    contains the rectification rotation (P = R K [I|t]), so x is projected with
    the full homogeneous P @ [x;1] / z. Passing R to undistortPoints here would
    apply the rotation a second time and corrupt the result.
    """
    x = cv2.undistortPoints(corners, C["K"], C["D"]).reshape(-1, 3)
    P = C["P"]
    Xh = np.hstack([x, np.ones((x.shape[0], 1))])
    Y = (P @ Xh.T).T
    Y = Y / Y[:, 2:3]
    return Y[:, :2]


class Ev(Node):
    def __init__(self):
        super().__init__("evidence_live")
        self.bridge = CvBridge()
        self.frames = 0
        self.start = time.time()
        self.last_centroid = None
        self.epipolar = []
        self.valid_frac = []
        self.disp_mean = []
        self.disp_median = []
        self.square = []
        self.plane_dev = []
        self.saved = 0
        subL = message_filters.Subscriber(self, Image, "/stereo/stereo_camera/left/image_raw")
        subR = message_filters.Subscriber(self, Image, "/stereo/stereo_camera/right/image_raw")
        sync = message_filters.ApproximateTimeSynchronizer([subL, subR], 10, slop=0.1)
        sync.registerCallback(self.cb)
        self.create_timer(5.0, self._log)
        self.get_logger().info(
            f"[evidence] loaded calib from {CALIB_DIR}; capturing up to "
            f"{N_FRAMES} poses from the sim stream ...")

    def _log(self):
        self.get_logger().info(
            f"[evidence] frames={self.frames}/{N_FRAMES} "
            f"(t={time.time()-self.start:.0f}s)")

    def cb(self, lm, rm):
        if self.frames >= N_FRAMES:
            self._finish()
            return
        try:
            Limg = self.bridge.imgmsg_to_cv2(lm, "bgr8")
            Rimg = self.bridge.imgmsg_to_cv2(rm, "bgr8")
        except Exception:
            return
        gL = cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY)
        gR = cv2.cvtColor(Rimg, cv2.COLOR_BGR2GRAY)
        okL, cL = find(gL)
        okR, cR = find(gR)
        if not (okL and okR):
            return
        # One capture per board pose: require the board centroid to have moved.
        centroid = np.mean(cL, axis=0).ravel()
        if self.last_centroid is not None:
            if float(np.linalg.norm(centroid - self.last_centroid)) < CENTROID_JUMP:
                if time.time() - self.start > MAX_WAIT_S:
                    self._finish()
                return
        self.last_centroid = centroid
        self.frames += 1
        n = self.frames

        rL = cv2.remap(gL, mapLx, mapLy, cv2.INTER_LINEAR)
        rR = cv2.remap(gR, mapRx, mapRy, cv2.INTER_LINEAR)
        disp = sbm.compute(rL, rR).astype(np.float32) / 16.0
        valid = disp >= 0
        self.valid_frac.append(float(valid.mean()))
        if valid.any():
            self.disp_mean.append(float(disp[valid].mean()))
            self.disp_median.append(float(np.median(disp[valid])))

        # (A) epipolar alignment of matched corners in the rectified views
        pl = rect_pix(cL, L)
        pr = rect_pix(cR, R)
        self.epipolar.append(np.abs(pl[:, 1] - pr[:, 1]))

        # (C) triangulate rectified corners -> 3-D; measure square size + coplanarity
        X = cv2.triangulatePoints(L["P"], R["P"], pl.T, pr.T)
        X = X[:3] / X[3:4]  # (3, N)
        dists = []
        Npts = X.shape[1]
        for i in range(Npts):
            if (i % COLS) != COLS - 1 and i + 1 < Npts:
                dists.append(np.linalg.norm(X[:, i + 1] - X[:, i]))
            if i + ROWS < Npts:
                dists.append(np.linalg.norm(X[:, i + ROWS] - X[:, i]))
        self.square.append(float(np.mean(dists)))
        pts = X.T
        centroid3 = pts.mean(axis=0)
        _, eigenvectors = cv2.PCACompute(pts.astype(np.float32), mean=None)
        normal = eigenvectors[-1]  # least-variance direction = plane normal
        self.plane_dev.append(float(np.abs((pts - centroid3) @ normal).max()))

        if n <= 3 or n % 6 == 0 or n == N_FRAMES:
            self._save(n, Limg, Rimg, rL, rR, disp, pl)
        if self.frames >= N_FRAMES:
            self._finish()

    def _save(self, n, Limg, Rimg, rL, rR, disp, pl):
        d8 = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        dv = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
        ov = cv2.cvtColor(rL, cv2.COLOR_GRAY2BGR)
        for i in range(0, len(pl), 6):
            y = int(pl[i, 1])
            cv2.line(ov, (0, y), (w - 1, y), (0, 255, 0), 1)
            cv2.circle(ov, (int(pl[i, 0]), y), 3, (0, 0, 255), -1)
        cv2.imwrite(os.path.join(OUT, f"frame_{n:02d}_L_R_disp.png"),
                    np.hstack([cv2.cvtColor(rL, cv2.COLOR_GRAY2BGR),
                               cv2.cvtColor(rR, cv2.COLOR_GRAY2BGR), dv]))
        cv2.imwrite(os.path.join(OUT, f"frame_{n:02d}_epipolar.png"), ov)
        cv2.imwrite(os.path.join(OUT, f"frame_{n:02d}_rawL.png"), Limg)
        cv2.imwrite(os.path.join(OUT, f"frame_{n:02d}_rawR.png"), Rimg)
        self.saved += 1

    def _finish(self):
        if getattr(self, "_done", False):
            return
        self._done = True
        ep = np.concatenate(self.epipolar) if self.epipolar else np.zeros(1)
        sq = np.array(self.square)
        pd = np.array(self.plane_dev)
        calib_base = float(abs(R["P"][0, 3]) / R["P"][0, 0])
        res = {
            "n_frames": int(self.frames),
            "epipolar_row_error_px": {
                "mean": float(ep.mean()), "median": float(np.median(ep)),
                "p95": float(np.percentile(ep, 95)), "max": float(ep.max()),
                "std": float(ep.std()),
            },
            "disparity": {
                "valid_frac_mean": float(np.mean(self.valid_frac)) if self.valid_frac else None,
                "mean_disp_mean": float(np.mean(self.disp_mean)) if self.disp_mean else None,
                "median_disp_mean": float(np.mean(self.disp_median)) if self.disp_median else None,
            },
            "triangulated_chessboard": {
                "true_square_m": TRUE_SQUARE,
                "recovered_square_m_mean": float(sq.mean()) if len(sq) else None,
                "recovered_square_m_std": float(sq.std()) if len(sq) else None,
                "scale_vs_true": float(sq.mean() / TRUE_SQUARE) if len(sq) else None,
                "max_plane_dev_m": float(pd.max()) if len(pd) else None,
            },
            "baseline": {
                "calibrated_m": calib_base,
                "physical_m": 0.06028,
                "scale": calib_base / 0.06028,
            },
        }
        with open(os.path.join(OUT, "evidence_summary.json"), "w") as f:
            json.dump(res, f, indent=2)
        self.get_logger().info("=== EVIDENCE SUMMARY (live) ===")
        self.get_logger().info(json.dumps(res, indent=2))
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Ev()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
