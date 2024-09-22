import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from example_interfaces.srv import AddTwoInts
import time
import board
import busio
from digitalio import DigitalInOut
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
        bno = BNO08X_I2C(i2c, address=0x4B)

        bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

        while True:
            time.sleep(0.1)

            self.get_logger().info("Acceleration:")
            accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
            self.get_logger().info("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
            self.get_logger().info("")

            self.get_logger().info("Gyro:")
            gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
            self.get_logger().info("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
            self.get_logger().info("")

            self.get_logger().info("Magnetometer:")
            mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
            self.get_logger().info("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
            self.get_logger().info("")

            self.get_logger().info("Linear Acceleration:")
            (
                linear_accel_x,
                linear_accel_y,
                linear_accel_z,
            ) = bno.linear_acceleration  # pylint:disable=no-member
            self.get_logger().info(
                "X: %0.6f  Y: %0.6f Z: %0.6f m/s^2"
                % (linear_accel_x, linear_accel_y, linear_accel_z)
            )
            self.get_logger().info("")

            self.get_logger().info("Rotation Vector Quaternion:")
            quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
            self.get_logger().info(
                "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
            )
            self.get_logger().info("")

            calibration_status = bno.calibration_status
            self.get_logger().info(
                "Magnetometer Calibration quality:" + 
                str(adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status]) + 
                " ("  + str(calibration_status) + ")"
            )
            
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()