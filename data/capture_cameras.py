import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2


class CameraGrabber(Node):
    def __init__(self):
        super().__init__("camera_grabber")
        self.images = {}
        self.sub_l = self.create_subscription(
            Image, "/stereo/stereo_camera/left/image_raw_raw", self.cb("left"), 1)
        self.sub_r = self.create_subscription(
            Image, "/stereo/stereo_camera/right/image_raw_raw", self.cb("right"), 1)

    def cb(self, name):
        def _cb(msg):
            if name not in self.images:
                enc = msg.encoding.lower()
                arr = np.frombuffer(msg.data, dtype=np.uint16 if "16" in enc else np.uint8)
                channels = msg.step // msg.width if msg.step else 3
                arr = arr.reshape(msg.height, msg.width, channels)
                if enc == "bgr8":
                    arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
                self.images[name] = arr
                self.get_logger().info(
                    f"got {name}: {arr.shape} enc={msg.encoding}")
        return _cb


def main():
    rclpy.init()
    node = CameraGrabber()
    import time
    start = time.time()
    while rclpy.ok() and len(node.images) < 2 and time.time() - start < 60:
        rclpy.spin_once(node, timeout_sec=0.5)
    paths = []
    for name in ("left", "right"):
        if name in node.images:
            p = f"/root/data/camera_{name}.png"
            cv2.imwrite(p, cv2.cvtColor(node.images[name], cv2.COLOR_RGB2BGR))
            paths.append(p)
            print(f"SAVED {p}")
        else:
            print(f"MISSING {name}", file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if len(paths) == 2 else 1


if __name__ == "__main__":
    sys.exit(main())
