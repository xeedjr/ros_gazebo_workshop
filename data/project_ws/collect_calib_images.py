#!/usr/bin/env python3
"""Collect annotated stereo calibration images to a folder.

Subscribes to /stereo_calib/{left,right}_annotated and saves every
frame as PNG under the output directory.  Runs until killed.
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageCollector(Node):
    def __init__(self, out_dir):
        super().__init__("calib_image_collector")
        self.out_dir = out_dir
        self.left_dir = os.path.join(out_dir, "left")
        self.right_dir = os.path.join(out_dir, "right")
        os.makedirs(self.left_dir, exist_ok=True)
        os.makedirs(self.right_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.n = 0
        self.create_subscription(
            Image, "/stereo_calib/left_annotated", self._on_left,
            qos_profile_system_default)
        self.create_subscription(
            Image, "/stereo_calib/right_annotated", self._on_right,
            qos_profile_system_default)
        self.get_logger().info(f"saving to {out_dir}")

    def _save(self, msg, d):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(os.path.join(d, f"{self.n:05d}.png"), img)

    def _on_left(self, msg):
        self._save(msg, self.left_dir)
        self.n += 1
        if self.n % 50 == 1:
            self.get_logger().info(f"saved {self.n} pairs")

    def _on_right(self, msg):
        self._save(msg, self.right_dir)


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else \
        "/root/data/project_ws/calib_images"
    rclpy.init()
    node = ImageCollector(out_dir)
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
