#!/usr/bin/env python3
"""Headless ROS2 stereo camera calibration node.

Drop-in replacement for the GUI stereo_calibrator from the
``camera_calibration`` package.  The same ``StereoCalibrator`` engine
is used; GUI buttons become ROS services and the image panels become
ROS image topics.

The ``StereoCalibrator.handle_msg()`` call returns a ``StereoDrawable``
whose ``lscrib`` / ``rscrib`` fields are the exact annotated BGR images
the GUI displays (corners drawn, rectified post-calibration).  This
node publishes them on ROS image topics and overlays a status string.

Services (std_srvs/srv/Trigger):
    /stereo_calib/calibrate   run stereo calibration
    /stereo_calib/save        save to /tmp/calibrationdata.tar.gz
    /stereo_calib/clear       clear accumulated samples
    /stereo_calib/cancel      shut the node down

Published topics:
    /stereo_calib/left_annotated   (sensor_msgs/Image)  left  view + corners
    /stereo_calib/right_annotated  (sensor_msgs/Image)  right view + corners
    /stereo_calib/status           (std_msgs/String)    progress string
    /stereo_calib/result           (std_msgs/String)    calibration report

Parameters (ROS 2):
    camera_name           str    calibration output name      (default "stereo_camera")
    pattern_cols          int    chessboard columns           (default 9)
    pattern_rows          int    chessboard rows              (default 6)
    square                float  square size in metres        (default 0.04)
    left_topic            str    left image topic
    right_topic           str    right image topic
    checkerboard_flags    int    cv2 chessboard flags         (default CALIB_CB_FAST_CHECK)
    max_chessboard_speed  float  max board speed px/s         (default -1 = disabled)
    publish_annotated     bool   publish annotated images     (default true)
    annotated_pub_rate    float  max Hz for annotated pubs    (default 5.0)
"""

import contextlib
import io
import sys
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import message_filters
from cv_bridge import CvBridge

sys.path.insert(0, "/opt/ros/jazzy/lib/python3.12/site-packages")
import camera_calibration.calibrator as _cal_mod  # noqa: E402
from camera_calibration.calibrator import (  # noqa: E402
    StereoCalibrator, ChessboardInfo, Patterns)

import numpy as np  # noqa: E402


def _sb_get_corners(img, board, refine=True, checkerboard_flags=0):
    """Replacement for _get_corners using findChessboardCornersSB.

    The classic cv2.findChessboardCorners used by camera_calibration fails
    on Gazebo-simulated images; the Shi-Tomasi-based SB variant is robust
    enough and matches what stereo_calibration_node already uses.
    """
    h, w = img.shape[:2]
    if len(img.shape) == 3 and img.shape[2] == 3:
        mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        mono = img
    try:
        ok, corners = cv2.findChessboardCornersSB(
            mono, (board.n_cols, board.n_rows))
    except AttributeError:
        ok, corners = cv2.findChessboardCorners(
            mono, (board.n_cols, board.n_rows),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE | checkerboard_flags)
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


# Patch the module-level function that get_corners() calls.
_cal_mod._get_corners = _sb_get_corners

# --- Fix: stereoRectify in-place P corruption (OpenCV 4.6 / ROS Jazzy) ------
# StereoCalibrator.set_alpha() passes the pre-allocated self.l.P / self.r.P as
# the P1/P2 *output* buffers of cv2.stereoRectify. In this OpenCV build that
# in-place write leaves the focal-length diagonal (fx, fy) garbage (observed
# fx ~1e5 instead of ~253) while the principal-point columns come out right, so
# the saved projection_matrix is wrong. Calling stereoRectify with *fresh*
# (None) output buffers on the same inputs is reliable, so after calibration we
# recompute R1/R2/P1/P2 that way and rebuild the rectify maps from the fixed P.
def recompute_stereo_rectify(calib, log):
    c = calib
    res = cv2.stereoRectify(
        c.l.intrinsics, c.l.distortion,
        c.r.intrinsics, c.r.distortion,
        c.size, c.R, c.T,
        None, None, None, None)
    c.l.R[:] = res[0]
    c.r.R[:] = res[1]
    c.l.P[:] = res[2]
    c.r.P[:] = res[3]
    try:
        cv2.initUndistortRectifyMap(
            c.l.intrinsics, c.l.distortion, c.l.R, c.l.P,
            c.size, cv2.CV_32FC1, c.l.mapx, c.l.mapy)
        cv2.initUndistortRectifyMap(
            c.r.intrinsics, c.r.distortion, c.r.R, c.r.P,
            c.size, cv2.CV_32FC1, c.r.mapx, c.r.mapy)
    except Exception as exc:  # maps only affect the rectified display
        log(f"recompute rectify maps: {exc}")


# compute_goodenough triggers "ready" at len(db) >= 40 or full param
# coverage; 40 is the hard floor used in the progress display.
_TARGET = 40


class StereoCalibHeadlessNode(Node):
    """Headless stereo calibrator exposing the GUI via ROS interfaces."""

    def __init__(self):
        super().__init__("stereo_calib_headless")

        self.declare_parameter("camera_name", "stereo_camera")
        self.declare_parameter("pattern_cols", 9)
        self.declare_parameter("pattern_rows", 6)
        self.declare_parameter("square", 0.04)
        # Unprocessed stereo frames straight from the ros_gz bridge (the
        # bridge's GZ_TO_ROS output), bypassing any republish/processing node.
        self.declare_parameter("left_topic",
                               "/stereo/stereo_camera/left/image_raw_raw")
        self.declare_parameter("right_topic",
                               "/stereo/stereo_camera/right/image_raw_raw")
        self.declare_parameter("checkerboard_flags",
                               int(cv2.CALIB_CB_FAST_CHECK))
        self.declare_parameter("max_chessboard_speed", -1.0)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("annotated_pub_rate", 5.0)

        camera_name = self.get_parameter("camera_name").value
        self.cols = int(self.get_parameter("pattern_cols").value)
        self.rows = int(self.get_parameter("pattern_rows").value)
        self.square = float(self.get_parameter("square").value)
        left_topic = self.get_parameter("left_topic").value
        right_topic = self.get_parameter("right_topic").value
        self.cb_flags = int(self.get_parameter("checkerboard_flags").value)
        max_speed = float(self.get_parameter("max_chessboard_speed").value)
        self._pub_ann = bool(self.get_parameter("publish_annotated").value)
        self._ann_rate = float(
            self.get_parameter("annotated_pub_rate").value)

        self.bridge = CvBridge()

        boards = [ChessboardInfo("chessboard", self.cols, self.rows,
                                 self.square)]
        self.calib = StereoCalibrator(
            boards, 0, 0, Patterns.Chessboard,
            name=camera_name,
            checkerboard_flags=self.cb_flags,
            max_chessboard_speed=max_speed)

        self.pub_left = self.create_publisher(
            Image, "/stereo_calib/left_annotated",
            qos_profile_system_default)
        self.pub_right = self.create_publisher(
            Image, "/stereo_calib/right_annotated",
            qos_profile_system_default)
        self.pub_status = self.create_publisher(
            String, "/stereo_calib/status", 10)
        self.pub_result = self.create_publisher(
            String, "/stereo_calib/result", 10)

        self.create_service(
            Trigger, "/stereo_calib/calibrate", self._srv_calibrate)
        self.create_service(
            Trigger, "/stereo_calib/save", self._srv_save)
        self.create_service(
            Trigger, "/stereo_calib/clear", self._srv_clear)
        self.create_service(
            Trigger, "/stereo_calib/cancel", self._srv_cancel)

        lsub = message_filters.Subscriber(
            self, Image, left_topic,
            qos_profile=qos_profile_system_default)
        rsub = message_filters.Subscriber(
            self, Image, right_topic,
            qos_profile=qos_profile_system_default)
        ts = message_filters.ApproximateTimeSynchronizer(
            [lsub, rsub], 4, slop=0.1)
        ts.registerCallback(self._on_stereo)

        self._t_last_ann = 0.0
        self._t_last_status = 0.0
        self._cancelled = False

        self.get_logger().info(
            f"headless stereo calibrator up  "
            f"camera={camera_name}  pattern={self.cols}x{self.rows}  "
            f"square={self.square} m  "
            f"left={left_topic}  right={right_topic}")

    # ── helpers ───────────────────────────────────────────────────────────

    def _status(self):
        n = len(self.calib.db)
        if self.calib.calibrated:
            state = "CALIBRATED"
        elif self.calib.goodenough:
            state = "READY"
        else:
            state = "COLLECTING"
        return f"[{state}] good={n}/{_TARGET}"

    def _overlay(self, bgr, text):
        img = bgr.copy()
        cv2.putText(img, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return img

    def _capture_report(self):
        """report() prints to stdout; capture it here."""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.calib.report()
        return buf.getvalue()

    # ── per-frame callback ────────────────────────────────────────────────

    def _on_stereo(self, left_msg, right_msg):
        if self._cancelled:
            return

        prev_n = len(self.calib.db)
        try:
            rv = self.calib.handle_msg((left_msg, right_msg))
        except Exception as exc:
            self.get_logger().warn(f"handle_msg: {exc}")
            return
        new_n = len(self.calib.db)
        new_sample = new_n > prev_n

        now = time.monotonic()

        if self._pub_ann and (
                new_sample
                or (now - self._t_last_ann) >= 1.0 / self._ann_rate):
            text = self._status()
            lm = self.bridge.cv2_to_imgmsg(
                self._overlay(rv.lscrib, text), "bgr8")
            rm = self.bridge.cv2_to_imgmsg(
                self._overlay(rv.rscrib, text), "bgr8")
            lm.header = left_msg.header
            rm.header = right_msg.header
            self.pub_left.publish(lm)
            self.pub_right.publish(rm)
            self._t_last_ann = now

        if now - self._t_last_status >= 1.0:
            self.pub_status.publish(String(data=self._status()))
            self._t_last_status = now

    # ── service handlers ──────────────────────────────────────────────────

    def _srv_calibrate(self, req, resp):
        if self.calib.calibrated:
            resp.success = False
            resp.message = (
                "Already calibrated; call /stereo_calib/clear first.")
            return resp
        if not self.calib.goodenough:
            resp.success = False
            resp.message = (
                f"Not enough samples "
                f"({len(self.calib.db)}/{_TARGET}).")
            return resp

        self.get_logger().info("Running stereo calibration ...")
        try:
            self.calib.do_calibration()
        except Exception as exc:
            resp.success = False
            resp.message = f"Calibration failed: {exc}"
            return resp

        # Fix the projection matrices corrupted by the in-pipeline stereoRectify
        # in-place write (OpenCV 4.6), before the report is captured and saved.
        recompute_stereo_rectify(self.calib, self.get_logger().warn)

        report = self._capture_report()
        self.get_logger().info("Stereo calibration complete.")
        self.pub_result.publish(String(data=report))
        resp.success = True
        resp.message = report
        return resp

    def _srv_save(self, req, resp):
        if not self.calib.calibrated:
            resp.success = False
            resp.message = (
                "Not calibrated yet; "
                "call /stereo_calib/calibrate first.")
            return resp
        try:
            self.calib.do_save()
        except Exception as exc:
            resp.success = False
            resp.message = f"Save failed: {exc}"
            return resp
        resp.success = True
        resp.message = "Saved /tmp/calibrationdata.tar.gz"
        return resp

    def _srv_clear(self, req, resp):
        self.calib.db.clear()
        self.calib.good_corners.clear()
        self.calib.goodenough = False
        self.get_logger().info("Cleared all accumulated samples.")
        self.pub_status.publish(String(data=self._status()))
        resp.success = True
        resp.message = "Cleared."
        return resp

    def _srv_cancel(self, req, resp):
        self._cancelled = True
        resp.success = True
        resp.message = "Shutting down."
        self.get_logger().info("Cancel requested.")
        threading.Timer(0.3, rclpy.shutdown).start()
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = StereoCalibHeadlessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
