import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from example_interfaces.srv import AddTwoInts
from sensor_msgs.msg import Imu

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.i = 0
        
        self.is_discover_allowed = False

        # ros2 service call /discover_map example_interfaces/AddTwoInts "{a: 1, b: 2}"
        self.srv = self.create_service(AddTwoInts, 'discover_map', self.discover_map_callback)

        self.topic_cmd_vel_nav = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.callback_cmd_vel,
            0)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 0)
        # Subscription to the marker array topic
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/contour_markers_sorted',  # Replace with your actual topic
            self.marker_callback,
            10
        )

        self.prevoius_used_markers = MarkerArray()

        self.navigator = BasicNavigator()

        #Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = 2.15
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        #navigator.waitUntilNav2Active()
        self.navigator.waitUntilNav2Active(localizer="bt_navigator")

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()

    def discover_map_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        self.is_discover_allowed = True

        return response

    def callback_cmd_vel(self, cmd_vel_msg_in):
#        self.get_logger().info('cmd_vel_received')

        cmd_vel_msg = TwistStamped()
        
        # Add current time and frame_id to the header
        cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg.header.frame_id = 'base_link'  # Adjust frame ID as needed

        # Set linear and angular velocities
        cmd_vel_msg.twist.linear = cmd_vel_msg_in.linear
        cmd_vel_msg.twist.angular = cmd_vel_msg_in.angular

        # Publish the message
        self.cmd_vel_pub.publish(cmd_vel_msg)
        # self.get_logger().info(f'Publishing TwistStamped: {cmd_vel_msg}')


    def marker_callback(self, msg: MarkerArray):

        if self.is_discover_allowed == False:
            return

        if len(msg.markers) == 0:
            return

        if self.navigator.isTaskComplete():
            # Process next marker

            self.get_logger().info('Loop to markers !!!!!!!!!!!!!!!!')

            current_marker = None
            for marker in msg.markers:
                if marker.pose.position.x not in [m.pose.position.x for m in self.prevoius_used_markers.markers]:
                    current_marker = marker
                    break

            if current_marker:
                self.get_logger().info('Go to to markers !!!!!!!!!!!!!!!!')

                # Create goal pose based on current marker
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = current_marker.pose.position.x
                goal_pose.pose.position.y = current_marker.pose.position.y
                goal_pose.pose.orientation.w = 1.0

                # sanity check a valid path exists
                # path = navigator.getPath(initial_pose, goal_pose)

                self.navigator.goToPose(goal_pose)

                self.prevoius_used_markers.markers.append(current_marker)
        else:
            # Wait for ending
            pass

    #ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
    # def add_two_ints_callback(self, request, response):
    #     response.sum = request.a + request.b
    #     self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))


    #     # Go to our demos first goal pose
    #     goal_pose = PoseStamped()
    #     goal_pose.header.frame_id = 'map'
    #     goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    #     goal_pose.pose.position.x = -2.0
    #     goal_pose.pose.position.y = -0.5
    #     goal_pose.pose.orientation.w = 1.0

    #     # sanity check a valid path exists
    #     # path = navigator.getPath(initial_pose, goal_pose)

    #     self.navigator.goToPose(goal_pose)

    #     i = 0
    #     while not self.navigator.isTaskComplete():
    #         ################################################
    #         #
    #         # Implement some code here for your application!
    #         #
    #         ################################################

    #         # Do something with the feedback
    #         i = i + 1
    #         feedback = navigator.getFeedback()
    #         if feedback and i % 5 == 0:
    #             print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #                 Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #                 + ' seconds.')

    #             # Some navigation timeout to demo cancellation
    #             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #                 navigator.cancelTask()

    #             # Some navigation request change to demo preemption
    #             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    #                 goal_pose.pose.position.x = -3.0
    #                 navigator.goToPose(goal_pose)

    #     # Do something depending on the return code
    #     result = navigator.getResult()
    #     if result == TaskResult.SUCCEEDED:
    #         print('Goal succeeded!')
    #     elif result == TaskResult.CANCELED:
    #         print('Goal was canceled!')
    #     elif result == TaskResult.FAILED:
    #         print('Goal failed!')
    #     else:
    #         print('Goal has an invalid return status!')

    #     navigator.lifecycleShutdown()

    #     return response





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