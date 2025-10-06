#!/usr/bin/env python3
"""Stereo fisheye rectified image publisher for ROS 2 Jazzy."""

from __future__ import annotations

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import yaml

#ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.02325 --no-service-check right:=/stereo/stereo_camera/right/image_raw left:=/stereo/stereo_camera/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/
#ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.0666 --no-service-check right:=/stereo/stereo_camera/right/image_raw left:=/stereo/stereo_camera/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/
#v4l2-ctl --device=/dev/video2 --list-formats-ext


class StereoCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("stereo_camera_publisher")

        # Parameters
        self.declare_parameter("camera_device", 2)
        self.declare_parameter("frame_id", "camera_left_optical_link")
        self.declare_parameter("topic_namespace", "")
        self.declare_parameter("publish_rate", 30.0)

        camera_device = self.get_parameter("camera_device").value
        frame_id = self.get_parameter("frame_id").value
        topic_ns = self.get_parameter("topic_namespace").value.rstrip("/")
        fallback_rate = self.get_parameter("publish_rate").value

        # Open camera
        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Unable to open camera {camera_device}")
            raise RuntimeError("Camera open failed")

        # Set raw mode for the camera
        #self.cap.set(cv2.CAP_PROP_FORMAT, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3200)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

        # Log camera parameters after opening
        self.get_logger().info(f"Camera opened: device={camera_device}")
        self.get_logger().info(f"Frame width: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
        self.get_logger().info(f"Frame height: {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        self.get_logger().info(f"FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")
        self.get_logger().info(f"FourCC: {int(self.cap.get(cv2.CAP_PROP_FOURCC))}")
        self.get_logger().info(f"Format: {self.cap.get(cv2.CAP_PROP_FORMAT)}")
        self.get_logger().info(f"Mode: {self.cap.get(cv2.CAP_PROP_MODE)}")
        self.get_logger().info(f"Brightness: {self.cap.get(cv2.CAP_PROP_BRIGHTNESS)}")
        self.get_logger().info(f"Contrast: {self.cap.get(cv2.CAP_PROP_CONTRAST)}")
        self.get_logger().info(f"Saturation: {self.cap.get(cv2.CAP_PROP_SATURATION)}")
        self.get_logger().info(f"Hue: {self.cap.get(cv2.CAP_PROP_HUE)}")
        self.get_logger().info(f"Gain: {self.cap.get(cv2.CAP_PROP_GAIN)}")
        self.get_logger().info(f"Exposure: {self.cap.get(cv2.CAP_PROP_EXPOSURE)}")
        self.get_logger().info(f"Auto Exposure: {self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)}")
        self.get_logger().info(f"White Balance: {self.cap.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U)}")
        self.get_logger().info(f"Auto White Balance: {self.cap.get(cv2.CAP_PROP_AUTO_WB)}")
        self.get_logger().info(f"Backend: {self.cap.getBackendName()}")


        full_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        full_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(self.cap.get(cv2.CAP_PROP_FPS)) or fallback_rate

        self.width = full_width // 2
        self.height = full_height


        self.left_info_file = self.get_camera_info('/root/data/project_ws/left.yaml')
        self.right_info_file = self.get_camera_info('/root/data/project_ws/right.yaml')

        def ns(name: str) -> str:
            return f"{topic_ns}/{name}".lstrip("/")

        # Publishers
        self.left_pub = self.create_publisher(Image, ns("/stereo/stereo_camera/left/image_raw"), 10)
        self.right_pub = self.create_publisher(Image, ns("/stereo/stereo_camera/right/image_raw"), 10)
        self.left_info_pub = self.create_publisher(CameraInfo, ns("/stereo/stereo_camera/left/camera_info"), 10)
        self.right_info_pub = self.create_publisher(CameraInfo, ns("/stereo/stereo_camera/right/camera_info"), 10)

        self.bridge = CvBridge()
        self.frame_id_left = "camera_left_optical_link"
        self.frame_id_right = "camera_right_optical_link"

        # CameraInfo
        self.left_info = self._make_camera_info("left")
        self.right_info = self._make_camera_info("right")

        period = 1.0 / fps if fps > 0.0 else 1.0 / fallback_rate
        self.timer = self.create_timer(period, self._capture_and_publish)




    def _make_camera_info(self, which: str) -> CameraInfo:
        msg = CameraInfo()

        if which == "left":
            msg.width = self.left_info_file["image_width"]
            msg.height = self.left_info_file["image_height"]
            msg.distortion_model = self.left_info_file["distortion_model"]
            msg.d = self.left_info_file["distortion_coefficients"]["data"]
            msg.k = self.left_info_file["camera_matrix"]["data"]
            msg.r = self.left_info_file["rectification_matrix"]["data"]
            msg.p = self.left_info_file["projection_matrix"]["data"]
        elif which == "right":
            msg.width = self.right_info_file["image_width"]
            msg.height = self.right_info_file["image_height"]
            msg.distortion_model = self.right_info_file["distortion_model"]
            msg.d = self.right_info_file["distortion_coefficients"]["data"]
            msg.k = self.right_info_file["camera_matrix"]["data"]
            msg.r = self.right_info_file["rectification_matrix"]["data"]
            msg.p = self.right_info_file["projection_matrix"]["data"]
        else:
            raise ValueError(f"Unknown camera '{which}'")

        return msg

    def get_camera_info(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        camera_info = {
            "image_width": data.get("image_width"),
            "image_height": data.get("image_height"),
            "camera_name": data.get("camera_name"),
            "camera_matrix": data.get("camera_matrix"),
            "distortion_model": data.get("distortion_model"),
            "distortion_coefficients": data.get("distortion_coefficients"),
            "rectification_matrix": data.get("rectification_matrix"),
            "projection_matrix": data.get("projection_matrix"),
        }
        return camera_info

    def _capture_and_publish(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Camera frame drop")
            return

        left_img = frame[:, :self.width]
        right_img = frame[:, self.width:]

        now = self.get_clock().now().to_msg()

        # Convert and publish
        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding="bgr8")
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding="bgr8")
        left_msg.header.stamp = right_msg.header.stamp = now
        left_msg.header.frame_id = self.frame_id_left
        right_msg.header.frame_id = self.frame_id_right

        self.left_info.header.stamp = now
        self.left_info.header.frame_id = self.frame_id_left
        self.right_info.header.stamp = now
        self.right_info.header.frame_id = self.frame_id_right

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.left_info_pub.publish(self.left_info)
        self.right_info_pub.publish(self.right_info)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

# Example usage:
# left_info = get_camera_info('/root/data/project_ws/left.yaml')
# print(left_info)

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StereoCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
