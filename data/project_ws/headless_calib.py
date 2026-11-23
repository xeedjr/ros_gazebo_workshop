"""Headless driver for camera_calibration's StereoCalibrator engine.

Feeds live synced /stereo/stereo_camera/{left,right}/image_raw pairs into the
same StereoCalibrator the `cameracalibrator` GUI uses, accumulates good
samples until goodenough, then calls do_calibration() + do_save() -- the
exact code path the GUI's CALIBRATE and SAVE buttons invoke. Writes the
official /tmp/calibrationdata.tar.gz.
"""
import os
import sys
import time

import cv2
import rclpy
import message_filters
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image

sys.path.insert(0, "/opt/ros/jazzy/lib/python3.12/site-packages")
from camera_calibration.calibrator import (  # noqa: E402
    StereoCalibrator, ChessboardInfo, Patterns)

TARGET = int(os.environ.get("TARGET", "40"))
MAXT = float(os.environ.get("MAXT", "240"))
DO_SAVE = os.environ.get("DO_SAVE", "1") == "1"
SQ = float(os.environ.get("SQUARE", "0.04"))
COLS = int(os.environ.get("COLS", "9"))
ROWS = int(os.environ.get("ROWS", "6"))


class Driver(Node):
    def __init__(self):
        super().__init__("headless_calib")
        boards = [ChessboardInfo("chessboard", COLS, ROWS, SQ)]
        # Same construction the GUI uses (camera_name default 'narrow_stereo',
        # fast-check on, max speed disabled).
        self.c = StereoCalibrator(
            boards, 0, 0, Patterns.Chessboard, name="narrow_stereo",
            checkerboard_flags=cv2.CALIB_CB_FAST_CHECK, max_chessboard_speed=-1.0)
        lsub = message_filters.Subscriber(
            self, Image, "/stereo/stereo_camera/left/image_raw",
            qos_profile=qos_profile_system_default)
        rsub = message_filters.Subscriber(
            self, Image, "/stereo/stereo_camera/right/image_raw",
            qos_profile=qos_profile_system_default)
        ts = message_filters.ApproximateTimeSynchronizer([lsub, rsub], 4, slop=0.1)
        ts.registerCallback(self.cb)
        self.t0 = time.time()
        self.last = self.t0

    def cb(self, l, r):
        try:
            self.c.handle_msg((l, r))
        except Exception as e:  # keep the loop alive
            print("handle_msg error: %r" % (e,), flush=True)
        now = time.time()
        if now - self.last > 5:
            self.last = now
            print("t=%3.0fs good=%d goodenough=%s" %
                  (now - self.t0, len(self.c.db), self.c.goodenough), flush=True)


def main():
    rclpy.init()
    node = Driver()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    t0 = time.time()
    while rclpy.ok():
        ex.spin_once(timeout_sec=0.05)
        if (node.c.goodenough or len(node.c.db) >= TARGET
                or (time.time() - t0) > MAXT):
            break
    n = len(node.c.db)
    print("=== accumulate done: good=%d goodenough=%s elapsed=%.0fs ==="
          % (n, node.c.goodenough, time.time() - t0), flush=True)
    if n == 0:
        print("NO GOOD SAMPLES -- cannot calibrate", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return
    node.c.do_calibration()          # prints report() with K/D/R/P + epi error
    print("calibrated=%s" % node.c.calibrated, flush=True)
    if DO_SAVE:
        node.c.do_save()
        print("tar exists:", os.path.exists("/tmp/calibrationdata.tar.gz"),
              flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
