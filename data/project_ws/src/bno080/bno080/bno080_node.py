import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header  # For timestamp and frame_id
import time
import board
import busio
from digitalio import DigitalInOut
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publishers for IMU and Magnetometer data
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_publisher_ = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Set up the I2C connection to the BNO08X sensor
        i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
        self.bno = BNO08X_I2C(i2c, address=0x4B)

        # Enable the BNO08X features
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

        # Create a timer to periodically publish the IMU and Magnetometer data
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # --- Publish IMU data ---
        imu_msg = Imu()

        # Add timestamp and frame_id
        imu_msg.header.stamp = self.get_clock().now().to_msg()  # Add current time
        imu_msg.header.frame_id = "imu_link"  # Set frame ID

        # Get IMU sensor data
        accel_x, accel_y, accel_z = self.bno.acceleration
        gyro_x, gyro_y, gyro_z = self.bno.gyro
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion

        # Fill the IMU message
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.orientation.x = quat_i
        imu_msg.orientation.y = quat_j
        imu_msg.orientation.z = quat_k
        imu_msg.orientation.w = quat_real

        # Publish the IMU message
        self.imu_publisher_.publish(imu_msg)
        self.get_logger().info(f"Published IMU data: {imu_msg}")

        # --- Publish Magnetometer data ---
        mag_msg = MagneticField()

        # Add timestamp and frame_id
        mag_msg.header.stamp = self.get_clock().now().to_msg()  # Add current time
        mag_msg.header.frame_id = "imu_link"  # Set frame ID

        # Get Magnetometer data
        mag_x, mag_y, mag_z = self.bno.magnetic

        # Fill the MagneticField message
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z

        # Publish the Magnetometer message
        self.mag_publisher_.publish(mag_msg)
        self.get_logger().info(f"Published Magnetometer data: {mag_msg}")

        calibration_status = self.bno.calibration_status
        self.get_logger().info(
            "Magnetometer Calibration quality:" + 
            str(adafruit_bno08x.REPORT_ACCURACY_STATUS[calibration_status]) + 
            " ("  + str(calibration_status) + ")"
        )
        
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
