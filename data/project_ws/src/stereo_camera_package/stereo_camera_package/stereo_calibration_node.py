#!/usr/bin/env python3
"""Stereo camera calibration node for the simulated stereo camera pipeline.

Sits in the image pipeline right after ``sim_stereo_camera_node``. It subscribes
to the synced left/right raw images (published by the Gazebo stereo cameras via
the ros_gz bridge), detects a 9x6 chessboard in both views, accumulates enough
diverse board poses (the board is teleported around by ``chessboard_mover``),
then runs cv2 stereo calibration and writes the recovered intrinsics to NEW yaml
files (``<output_dir>/left_sim.yaml`` / ``right_sim.yaml``) in exactly the same
format as the existing ``left.yaml`` / ``right.yaml``.

It also writes a human-readable summary to ``<output_dir>/stereo_calib_result.txt``
and keeps a few sample image pairs under ``<output_dir>/images/``.
"""

from __future__ import annotations

import os
import time

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters


def _mat_block(arr: np.ndarray) -> dict:
    """Format an ndarray as the {rows, cols, data: [flat]} block used in the
    existing left.yaml / right.yaml calibration files."""
    arr = np.asarray(arr, dtype=float)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    return {
        "rows": int(arr.shape[0]),
        "cols": int(arr.shape[1]),
        "data": [float(v) for v in arr.reshape(-1)],
    }


def _distortion_block(dist: np.ndarray) -> dict:
    d = np.asarray(dist, dtype=float).reshape(-1)
    if d.size < 5:
        d = np.concatenate([d, np.zeros(5 - d.size)])
    elif d.size > 5:
        d = d[:5]
    return {"rows": 1, "cols": 5, "data": [float(v) for v in d]}


class StereoCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("stereo_calibration_node")

        self.declare_parameter("pattern_cols", 9)
        self.declare_parameter("pattern_rows", 6)
        self.declare_parameter("square", 0.04)
        self.declare_parameter("min_poses", 16)
        self.declare_parameter("max_capture_time", 120.0)
        self.declare_parameter("output_dir",
                               "/root/data/project_ws/calibration_sim")
        self.declare_parameter("save_sample_images", True)
        self.declare_parameter("min_record_interval", 0.4)
        self.declare_parameter("centroid_jump_px", 10.0)
        self.declare_parameter("left_topic",
                               "/stereo/stereo_camera/left/image_raw")
        self.declare_parameter("right_topic",
                               "/stereo/stereo_camera/right/image_raw")

        self.cols = int(self.get_parameter("pattern_cols").value)
        self.rows = int(self.get_parameter("pattern_rows").value)
        self.square = float(self.get_parameter("square").value)
        self.min_poses = int(self.get_parameter("min_poses").value)
        self.max_capture_time = float(self.get_parameter("max_capture_time").value)
        self.output_dir = self.get_parameter("output_dir").value
        self.save_samples = bool(self.get_parameter("save_sample_images").value)
        self.min_record_interval = float(
            self.get_parameter("min_record_interval").value)
        self.centroid_jump_px = float(self.get_parameter("centroid_jump_px").value)
        left_topic = self.get_parameter("left_topic").value
        right_topic = self.get_parameter("right_topic").value

        self.bridge = CvBridge()

        # Canonical 3-D object points for the chessboard (board local frame).
        pattern = np.zeros((self.cols * self.rows, 3), dtype=np.float32)
        pattern[:, :2] = (np.mgrid[0:self.cols, 0:self.rows].T
                          .reshape(-1, 2) * self.square)
        pattern[:, 2] = 0.0
        self.objp = pattern

        self.left_pts: list = []
        self.right_pts: list = []
        self.image_size = None
        self.capture_start = None
        self.done = False
        self.last_centroid = None
        self.last_record_time = 0.0
        self.detected_frames = 0

        left_sub = message_filters.Subscriber(self, Image, left_topic)
        right_sub = message_filters.Subscriber(self, Image, right_topic)
        sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], 10, slop=0.05)
        sync.registerCallback(self._on_stereo)

        self.create_timer(1.0, self._check)

        self.get_logger().info(
            f"stereo_calibration_node up: pattern {self.cols}x{self.rows}, "
            f"square={self.square} m, min_poses={self.min_poses}, "
            f"out={self.output_dir}")

    # ------------------------------------------------------------------ detect
    def _find(self, gray: np.ndarray):
        # Prefer the Shi-Tomasi-based detector: it stays reliable when the
        # board is small / far / strongly tilted, where the classic adaptive
        # threshold detector drops out.
        if hasattr(cv2, "findChessboardCornersSB"):
            # OpenCV 4.6's findChessboardCornersSB takes no `criteria` keyword
            # (it refines internally); pass only image + pattern size. Return
            # immediately so a frame it misses is simply skipped, never fatal.
            ok, corners = cv2.findChessboardCornersSB(
                gray, (self.cols, self.rows))
            return (bool(ok), corners)
        # Fallback: classic detector with default flags. OpenCV >=4.5 removed
        # CALIB_CB_NORMAL_CHECK (now the default behaviour), so do not pass it.
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        ok, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows))
        if not ok:
            return False, None
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return True, corners

    def _on_stereo(self, left_msg: Image, right_msg: Image) -> None:
        if self.done:
            return
        try:
            left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        if self.image_size is None:
            self.image_size = (left.shape[1], left.shape[0])
            self.capture_start = time.time()
            self.get_logger().info(
                f"first stereo frame received, image size "
                f"{self.image_size[0]}x{self.image_size[1]}")

        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        ok_l, corners_l = self._find(gray_l)
        ok_r, corners_r = self._find(gray_r)
        if not (ok_l and ok_r):
            return

        self.detected_frames += 1
        if self.detected_frames % 30 == 1:
            self.get_logger().info(
                f"board detected in {self.detected_frames} stereo frames "
                f"so far, poses recorded={len(self.left_pts)}")

        # Record a pose when the board has moved (teleport) so the stored poses
        # stay diverse instead of duplicating one stationary pose. Measure the
        # 2-D displacement of the board's corner centroid (Euclidean).
        now = time.time()
        centroid = np.mean(corners_l, axis=0).ravel()  # shape (2,)
        moved = self.last_centroid is None or \
            float(np.linalg.norm(centroid - self.last_centroid)) \
            > self.centroid_jump_px
        if moved and (now - self.last_record_time) >= self.min_record_interval:
            self.left_pts.append(corners_l)
            self.right_pts.append(corners_r)
            self.last_centroid = centroid
            self.last_record_time = now
            n = len(self.left_pts)
            self.get_logger().info(
                f"board pose #{n} captured "
                f"(detected_frames={self.detected_frames}, n={n}/{self.min_poses})")
            if self.save_samples and n <= 6:
                self._save_sample(n, left, right)

    def _save_sample(self, n: int, left: np.ndarray, right: np.ndarray) -> None:
        img_dir = os.path.join(self.output_dir, "images")
        try:
            os.makedirs(img_dir, exist_ok=True)
            cv2.imwrite(os.path.join(img_dir, f"left_{n:03d}.png"), left)
            cv2.imwrite(os.path.join(img_dir, f"right_{n:03d}.png"), right)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"could not save sample images: {exc}")

    # --------------------------------------------------------------- trigger
    def _check(self) -> None:
        if self.done or not self.left_pts:
            return
        enough = len(self.left_pts) >= self.min_poses
        timed_out = ((time.time() - self.capture_start)
                     > self.max_capture_time)
        if enough or (timed_out and len(self.left_pts) >= 6):
            self._calibrate()
        elif timed_out:
            self.done = True
            self.get_logger().error(
                f"capture window ended with only {len(self.left_pts)} poses "
                f"(< 6); aborting calibration")

    # -------------------------------------------------------------- calibrate
    def _stereo_rms(self, obj, lp, rp, Kl, dl, rvl, tvl, Kr, dr, rvr, tvr) -> float:
        total, count = 0.0, 0
        for i in range(len(lp)):
            pl, _ = cv2.projectPoints(obj[i], rvl[i], tvl[i], Kl, dl)
            pr, _ = cv2.projectPoints(obj[i], rvr[i], tvr[i], Kr, dr)
            # Detected corners come back as (N,1,2) float32 while projected
            # points are (N,1,2) float32 too; normalise both to (N,2) float64
            # so the Euclidean error is well-defined (avoids cv2.norm's strict
            # same-type/same-shape requirement).
            pl = pl.reshape(-1, 2).astype(np.float64)
            pr = pr.reshape(-1, 2).astype(np.float64)
            li = lp[i].reshape(-1, 2).astype(np.float64)
            ri = rp[i].reshape(-1, 2).astype(np.float64)
            total += float(np.linalg.norm(li - pl))
            total += float(np.linalg.norm(ri - pr))
            count += 2
        return total / max(count, 1)

    def _calibrate(self) -> None:
        self.done = True
        w, h = self.image_size
        obj = [self.objp] * len(self.left_pts)
        self.get_logger().info(
            f"calibrating with {len(self.left_pts)} poses, image {w}x{h} ...")

        try:
            _, Kl, dl, rvl, tvl = cv2.calibrateCamera(
                obj, self.left_pts, (w, h), None, None)
            _, Kr, dr, rvr, tvr = cv2.calibrateCamera(
                obj, self.right_pts, (w, h), None, None)
            # OpenCV 4.x stereoCalibrate returns 9 values:
            #   rms, K1, D1, K2, D2, R, T, E, F
            # Use the (possibly refined) K/D it returns downstream.
            _, Kl, dl, Kr, dr, Rrel, T, E, F = cv2.stereoCalibrate(
                obj, self.left_pts, self.right_pts,
                Kl, dl, Kr, dr, (w, h))
            # OpenCV 4.x stereoRectify returns 7 values:
            #   R1, R2, P1, P2, Q, roi1, roi2
            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
                Kl, dl, Kr, dr, (w, h), Rrel, T,
                flags=cv2.CALIB_ZERO_DISPARITY)
        except cv2.error as exc:
            self.get_logger().error(f"cv2 calibration failed: {exc}")
            return

        rms = self._stereo_rms(
            obj, self.left_pts, self.right_pts,
            Kl, dl, rvl, tvl, Kr, dr, rvr, tvr)

        os.makedirs(self.output_dir, exist_ok=True)
        left_yaml = os.path.join(self.output_dir, "left_sim.yaml")
        right_yaml = os.path.join(self.output_dir, "right_sim.yaml")

        left_doc = {
            "image_width": int(w),
            "image_height": int(h),
            "camera_name": "stereo_camera/left",
            "camera_matrix": _mat_block(Kl),
            "distortion_model": "plumb_bob",
            "distortion_coefficients": _distortion_block(dl),
            "rectification_matrix": _mat_block(R1),
            "projection_matrix": _mat_block(P1),
        }
        right_doc = {
            "image_width": int(w),
            "image_height": int(h),
            "camera_name": "stereo_camera/right",
            "camera_matrix": _mat_block(Kr),
            "distortion_model": "plumb_bob",
            "distortion_coefficients": _distortion_block(dr),
            "rectification_matrix": _mat_block(R2),
            "projection_matrix": _mat_block(P2),
        }
        with open(left_yaml, "w") as f:
            yaml.safe_dump(left_doc, f, sort_keys=False)
        with open(right_yaml, "w") as f:
            yaml.safe_dump(right_doc, f, sort_keys=False)

        # Human-readable summary (matches the style of calibration.txt).
        lines = [
            "**** Sim stereo calibration ****",
            f"poses: {len(self.left_pts)}",
            f"image: {w}x{h}",
            f"Stereo RMS re-projection error: {rms:.6f}",
            "",
            "Left:",
            f"K = {np.array2string(Kl, precision=6)}",
            f"D = {np.array2string(np.asarray(dl).reshape(-1), precision=8)}",
            "",
            "Right:",
            f"K = {np.array2string(Kr, precision=6)}",
            f"D = {np.array2string(np.asarray(dr).reshape(-1), precision=8)}",
            "",
            f"R (relative) = {np.array2string(Rrel, precision=8)}",
            f"T = {np.array2string(T, precision=8)}",
            f"baseline (m) = {float(np.linalg.norm(T)):.6f}",
            "",
            "Left P  = "
            + ", ".join(f"{v:.6f}" for v in P1.reshape(-1)),
            "Right P = "
            + ", ".join(f"{v:.6f}" for v in P2.reshape(-1)),
            "",
            f"saved: {left_yaml}",
            f"saved: {right_yaml}",
        ]
        result_txt = os.path.join(self.output_dir, "stereo_calib_result.txt")
        with open(result_txt, "w") as f:
            f.write("\n".join(lines) + "\n")

        self.get_logger().info("---- stereo calibration complete ----")
        self.get_logger().info(f"RMS reprojection error: {rms:.4f}")
        self.get_logger().info(
            f"Left  K = [[{Kl[0,0]:.2f}, 0, {Kl[0,2]:.2f}], "
            f"[0, {Kl[1,1]:.2f}, {Kl[1,2]:.2f}]]")
        self.get_logger().info(
            f"Right K = [[{Kr[0,0]:.2f}, 0, {Kr[0,2]:.2f}], "
            f"[0, {Kr[1,1]:.2f}, {Kr[1,2]:.2f}]]")
        self.get_logger().info(
            f"baseline = {float(np.linalg.norm(T)):.4f} m "
            f"(sim stereo_tx_right ~ -15.26 px)")
        self.get_logger().info(f"saved: {left_yaml}")
        self.get_logger().info(f"saved: {right_yaml}")
        self.get_logger().info(f"saved: {result_txt}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StereoCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
