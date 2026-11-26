#!/usr/bin/env python3
"""Sim stereo camera republisher with calibration from YAML files for ROS 2 Jazzy.

Publishes final /stereo/stereo_camera/{left,right}/image_raw + camera_info topics.
Uses camera_info from calibration YAML files (calibration_sim/left_sim.yaml / right_sim.yaml)
instead of passing through Gazebo's camera_info. Corrects frame_id for RTAB-Map.
"""

from __future__ import annotations

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


def _load_calib(path: str) -> dict | None:
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f)
    except (OSError, yaml.YAMLError):
        return None


class SimStereoCameraNodeCalib(Node):
    def __init__(self) -> None:
        super().__init__("sim_stereo_camera_publisher_calib")

        # Parameters
        self.declare_parameter("stereo_tx_right", -15.256576)
        self.declare_parameter("frame_id_left", "camera_left_optical_link")
        self.declare_parameter("frame_id_right", "camera_right_optical_link")
        self.declare_parameter(
            "left_calib_file",
            "/root/data/project_ws/calibration_sim/left_sim.yaml")
        self.declare_parameter(
            "right_calib_file",
            "/root/data/project_ws/calibration_sim/right_sim.yaml")

        self.stereo_tx_right = float(
            self.get_parameter("stereo_tx_right").value)
        self.frame_id_left = self.get_parameter("frame_id_left").value
        self.frame_id_right = self.get_parameter("frame_id_right").value

        self.left_calib = _load_calib(
            self.get_parameter("left_calib_file").value)
        self.right_calib = _load_calib(
            self.get_parameter("right_calib_file").value)
        for side, calib in (("left", self.left_calib),
                            ("right", self.right_calib)):
            if calib is None:
                self.get_logger().warning(
                    f"{side} calibration yaml not loaded, "
                    "falling back to Gazebo camera_info pass-through")
            else:
                self.get_logger().info(f"{side} calibration yaml loaded")

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

    def _make_camera_info(self, msg: CameraInfo, frame_id: str,
                          calib: dict | None, tx: float) -> CameraInfo:
        out = CameraInfo()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = frame_id
        if calib is not None:
            out.width = int(calib["image_width"])
            out.height = int(calib["image_height"])
            out.distortion_model = calib["distortion_model"]
            out.d = [float(v) for v in calib["distortion_coefficients"]["data"]]
            out.k = [float(v) for v in calib["camera_matrix"]["data"]]
            out.r = [float(v) for v in calib["rectification_matrix"]["data"]]
            out.p = [float(v) for v in calib["projection_matrix"]["data"]]
            return out
        out.width = msg.width
        out.height = msg.height
        out.k = list(msg.k)
        out.distortion_model = ""
        out.d = []
        out.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        fx = float(out.k[0])
        fy = float(out.k[4])
        cx = float(out.k[2])
        cy = float(out.k[6])
        out.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, tx]
        return out

    def _left_image_cb(self, msg: Image) -> None:
        msg.header.frame_id = self.frame_id_left
        self.left_image_pub.publish(msg)

    def _right_image_cb(self, msg: Image) -> None:
        msg.header.frame_id = self.frame_id_right
        self.right_image_pub.publish(msg)

    def _left_info_cb(self, msg: CameraInfo) -> None:
        self.left_info_pub.publish(
            self._make_camera_info(
                msg, self.frame_id_left, self.left_calib, 0.0))

    def _right_info_cb(self, msg: CameraInfo) -> None:
        self.right_info_pub.publish(
            self._make_camera_info(
                msg, self.frame_id_right, self.right_calib,
                self.stereo_tx_right))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimStereoCameraNodeCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()