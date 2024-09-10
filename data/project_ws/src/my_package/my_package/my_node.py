import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import math

class ImuToCSV(Node):
    def __init__(self):
        super().__init__('imu_to_csv_node')
        # Subscribe to the IMU topic
        self.subscription = self.create_subscription(Imu, '/imu_sensor_broadcaster/imu', self.imu_callback, 10)
        self.subscription  # prevent unused variable warning
        self.csv_file = open('imu_readings.csv', mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['n_samples', 'gyr_x', 'gyr_y', 'gyr_z', 'acc_x', 'acc_y', 'acc_z'])
        self.sample_count = 0
    
    def imu_callback(self, msg):
        # Extract gyroscope and accelerometer data
        # Extract gyroscope data and convert from rad/s to deg/s
        gyr_x = math.degrees(msg.angular_velocity.x)
        gyr_y = math.degrees(msg.angular_velocity.y)
        gyr_z = math.degrees(msg.angular_velocity.z)

        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Write data to CSV
        self.csv_writer.writerow([self.sample_count, gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z])
        self.sample_count += 1
    
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ImuToCSV()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()