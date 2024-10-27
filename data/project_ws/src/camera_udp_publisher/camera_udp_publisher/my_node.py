import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSHistoryPolicy
import cv2

import struct

class ImageUDPClient(Node):
    def __init__(self):
        super().__init__('image_udp_client')
        
        # Define QoS profile for BestEffort
        qos_profile = QoSProfile(depth=2)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        self.subscription = self.create_subscription(
            Image,
            '/diff_drive/camera',  # Change to your image topic
            self.listener_callback,
            qos_profile)
        self.bridge = CvBridge()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('127.0.0.1', 55005))
        self.server_socket.listen(1)
        print(f'Server listening on')
        self.conn, _ = self.server_socket.accept()
        print('Client connected')

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format

        header = struct.pack(">IIIIB", msg.height, msg.width, len(msg.data), msg.step, msg.is_bigendian )


        self.conn.sendall(header)
        self.conn.sendall(bytes(msg.data))
        
        # self.get_logger().info('Sent image via UDP ' + str(msg.height) + str(msg.width)+ str(len(msg.data))+ str( msg.step)+ str( msg.is_bigendian ))

def main(args=None):
    rclpy.init(args=args)
    client = ImageUDPClient()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
