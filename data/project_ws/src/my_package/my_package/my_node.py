import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from example_interfaces.srv import AddTwoInts
from sensor_msgs.msg import Imu

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.i = 0
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.topic_cmd_vel_nav = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.callback_cmd_vel,
            0)
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_base_controller/cmd_vel_unstamped', 0)

        # Subscriber to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu_sensor_broadcaster/imu',  # Adjust the topic name if necessary
            self.imu_callback,
            10)

        # Publisher for the calibrated IMU data
        self.publisher_ = self.create_publisher(Imu, '/imu_sensor_broadcaster/imu/calibrated', 10)

        # Variables to store IMU data for calibration
        self.accel_x_samples = []
        self.accel_y_samples = []
        self.accel_z_samples = []
        self.gyro_x_samples = []
        self.gyro_y_samples = []
        self.gyro_z_samples = []

        self.sample_count = 0
        self.calibrated = False
        self.mean_accel_x = 0.0
        self.mean_accel_y = 0.0
        self.mean_accel_z = 0.0
        self.mean_gyro_x = 0.0
        self.mean_gyro_y = 0.0
        self.mean_gyro_z = 0.0

    def imu_callback(self, msg: Imu):
        if not self.calibrated:
            # Collect first 1000 samples for calibration
            self.accel_x_samples.append(msg.linear_acceleration.x)
            self.accel_y_samples.append(msg.linear_acceleration.y)
            self.accel_z_samples.append(msg.linear_acceleration.z)
            self.gyro_x_samples.append(msg.angular_velocity.x)
            self.gyro_y_samples.append(msg.angular_velocity.y)
            self.gyro_z_samples.append(msg.angular_velocity.z)

            self.sample_count += 1

            if self.sample_count == 1000:
                # Calculate the mean of the first 1000 samples
                self.mean_accel_x = sum(self.accel_x_samples) / 1000.0
                self.mean_accel_y = sum(self.accel_y_samples) / 1000.0
                self.mean_accel_z = sum(self.accel_z_samples) / 1000.0
                self.mean_gyro_x = sum(self.gyro_x_samples) / 1000.0
                self.mean_gyro_y = sum(self.gyro_y_samples) / 1000.0
                self.mean_gyro_z = sum(self.gyro_z_samples) / 1000.0

                self.get_logger().info('Calibration complete')
                self.calibrated = True
        else:
            # Apply the mean values to calibrate the data
            calibrated_msg = Imu()
            calibrated_msg.header = msg.header

            # Subtract the mean from the raw IMU data
            calibrated_msg.linear_acceleration.x = msg.linear_acceleration.x
            calibrated_msg.linear_acceleration.y = msg.linear_acceleration.y
            calibrated_msg.linear_acceleration.z = msg.linear_acceleration.z
            calibrated_msg.angular_velocity.x = msg.angular_velocity.x - self.mean_gyro_x
            calibrated_msg.angular_velocity.y = msg.angular_velocity.y - self.mean_gyro_y
            calibrated_msg.angular_velocity.z = msg.angular_velocity.z - self.mean_gyro_z

            # Publish the calibrated data
            self.publisher_.publish(calibrated_msg)

    def callback_cmd_vel(self, cmd_vel_msg):
#        self.get_logger().info('cmd_vel_received')

        self.cmd_vel_pub.publish(cmd_vel_msg)

    #ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))


        navigator = BasicNavigator()

        #Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = 2.15
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        navigator.setInitialPose(initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        #navigator.waitUntilNav2Active()
        navigator.waitUntilNav2Active(localizer="bt_navigator")

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()

        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = -2.0
        goal_pose.pose.position.y = -0.5
        goal_pose.pose.orientation.w = 1.0

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        navigator.lifecycleShutdown()

        return response





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