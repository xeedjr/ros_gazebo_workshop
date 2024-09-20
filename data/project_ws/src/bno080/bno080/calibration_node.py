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

        i2c = busio.I2C(board.SCL, board.SDA)
        bno = BNO08X_I2C(i2c, debug=False, address=0x4B)

        bno.begin_calibration()
        # TODO: UPDATE UART/SPI
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)
        start_time = time.monotonic()
        calibration_good_at = None
        while True:
            time.sleep(0.1)

            self.get_logger().info("Magnetometer:")
            mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
            self.get_logger().info("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
            self.get_logger().info("")

            self.get_logger().info("Game Rotation Vector Quaternion:")
            (
                game_quat_i,
                game_quat_j,
                game_quat_k,
                game_quat_real,
            ) = bno.game_quaternion  # pylint:disable=no-member
            self.get_logger().info(
                "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f"
                % (game_quat_i, game_quat_j, game_quat_k, game_quat_real)
            )
            calibration_status = bno.calibration_status
            self.get_logger().info(
                "Magnetometer Calibration quality:" + 
                str(adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status]) + 
                " ("  + str(calibration_status) + ")"
            )
            if not calibration_good_at and calibration_status >= 2:
                calibration_good_at = time.monotonic()
            if calibration_good_at and (time.monotonic() - calibration_good_at > 5.0):
                input_str = input("\n\nEnter S to save or anything else to continue: ")
                if input_str.strip().lower() == "s":
                    bno.save_calibration_data()
                    break
                calibration_good_at = None
            self.get_logger().info("**************************************************************")

        self.get_logger().info("calibration done")



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