import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/diff_drive/camera',  # Replace with your actual topic
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.image_received = False
        self.count = 10

    # def listener_callback(self, msg):
    #     if not self.image_received:  # Ensure the node exits after saving the first image
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    #         timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    #         filename = f'{timestamp}.png'
    #         cv2.imwrite(filename, cv_image)
    #         self.get_logger().info(f"Saved image as {filename}")
    #         self.image_received = True
    #         rclpy.shutdown()  # Stop the node after saving the image
    def listener_callback(self, msg):
        if (self.count == 0):
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            filename = f'{timestamp}.png'
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved image as {filename}")
            self.count = 10
        else:
            self.count = self.count -1;

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
