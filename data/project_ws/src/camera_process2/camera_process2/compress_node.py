import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge


#ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/compressed --remap out:=image_raw_uncompressed
class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')

        self.bridge = CvBridge()

        # Subscribe to the raw camera image
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        # Publisher for the compressed image
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/camera/compressed',
            10)

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Encode the OpenCV image as JPEG
            _, jpeg_image = cv2.imencode('.jpg', cv_image)

            # Create a CompressedImage ROS message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = jpeg_image.tobytes()

            # Publish the compressed image
            self.image_pub.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to compress image: {e}")

def main(args=None):
    rclpy.init(args=args)
    compressor_node = ImageCompressor()
    rclpy.spin(compressor_node)
    compressor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
