#!/usr/bin/env python3
"""Sim stereo camera republisher for ROS 2 Jazzy.

Mirrors stereo_camera_node in real mode: single source of the final
/stereo/stereo_camera/{left,right}/image_raw + camera_info topics. The
ros_gz bridge feeds this node the internal *_raw topics; the node sets the
image header.frame_id (ros_gz_bridge does not set it; RTAB-Map needs
camera_left_optical_link / camera_right_optical_link for TF) and uses the
CameraInfo from the bridge directly (with corrected frame_id).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class SimStereoCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_stereo_camera_publisher")

        # Parameters
        self.declare_parameter("frame_id_left", "camera_left_optical_link")
        self.declare_parameter("frame_id_right", "camera_right_optical_link")

        self.frame_id_left = self.get_parameter("frame_id_left").value
        self.frame_id_right = self.get_parameter("frame_id_right").value

        # Publishers (final topics consumed by image_proc / disparity / rtabmap)
        self.left_image_pub = self.create_publisher(
            Image, "/stereo/stereo_camera/left/image_raw", 10)
        self.right_image_pub = self.create_publisher(
            Image, "/stereo/stereo_camera/right/image_raw", 10)
        self.left_info_pub = self.create_publisher(
            CameraInfo, "/stereo/stereo_camera/left/camera_info", 10)
        self.right_info_pub = self.create_publisher(
            CameraInfo, "/stereo/stereo_camera/right/camera_info", 10)

        # Subscribers (internal raw topics fed by the ros_gz bridge)
        self.create_subscription(
            Image, "/stereo/stereo_camera/left/image_raw_raw",
            self._left_image_cb, 10)
        self.create_subscription(
            Image, "/stereo/stereo_camera/right/image_raw_raw",
            self._right_image_cb, 10)
        self.create_subscription(
            CameraInfo, "/stereo/stereo_camera/left/camera_info_raw",
            self._left_info_cb, 10)
        self.create_subscription(
            CameraInfo, "/stereo/stereo_camera/right/camera_info_raw",
            self._right_info_cb, 10)

    def _left_image_cb(self, msg: Image) -> None:
        msg.header.frame_id = self.frame_id_left
        self.left_image_pub.publish(msg)

    def _right_image_cb(self, msg: Image) -> None:
        msg.header.frame_id = self.frame_id_right
        self.right_image_pub.publish(msg)

    def _left_info_cb(self, msg: CameraInfo) -> None:
        msg.header.frame_id = self.frame_id_left
        self.left_info_pub.publish(msg)

    def _right_info_cb(self, msg: CameraInfo) -> None:
        msg.header.frame_id = self.frame_id_right
        self.right_info_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimStereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
