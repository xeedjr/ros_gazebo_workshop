import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import math

class MapProcessor(Node):

    def __init__(self):
        super().__init__('map_processor_node')

        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.process_map, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'processed_map', 10)
        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(MarkerArray, 'contour_markers', 10)
        self.additional_map_publisher = self.create_publisher(OccupancyGrid, 'additional_map', 10)
        self.marker_sorted_publisher = self.create_publisher(MarkerArray, 'contour_markers_sorted', 10)

        # Create a buffer and listener for tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscription to the marker array topic
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/contour_markers',  # Replace with your actual topic
            self.marker_callback,
            10
        )

    def marker_callback(self, msg: MarkerArray):
        self.marker_distances = self.get_marker_distances(msg)
        #self.get_logger().info(f"Markers Distamces: " + str(self.marker_distances))
        # self.sorted_markers = self.sort_markers_by_distance(self.marker_distances)
        # self.get_logger().info(f"Markers Distamces: " + str(self.sorted_markers))
        self.sort_markers_by_distance()
        #self.get_logger().info(f"Markers Distamces: " + str(self.marker_distances))
        self.publish_contour_markers_sorted()


    def get_marker_distances(self, marker_array):
        """Function to return list of marker and distance to base_link"""
        result = []
        try:
            # Get the current transform from base_link to map
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'base_link',  # Source frame
                rclpy.time.Time() 
            )

            base_x = trans.transform.translation.x
            base_y = trans.transform.translation.y
            base_z = trans.transform.translation.z

            # self.get_logger().info(f"base X: {base_x} Y: {base_y}")

            # Calculate distance for each marker
            for marker in marker_array.markers:
                marker_x = marker.pose.position.x
                marker_y = marker.pose.position.y
                marker_z = marker.pose.position.z

                # Compute Euclidean distance
                distance = math.sqrt(
                    (marker_x - base_x) ** 2 + 
                    (marker_y - base_y) ** 2 +
                    (marker_z - base_z) ** 2
                )

                # Append marker and distance to the result list
                result.append({
                    'original_marker': marker,
                    'distance': distance
                })

        except Exception as e:
            self.get_logger().warn(f'Could not transform base_link to map: {e}')

        return result

    def sort_markers_by_distance(self):
        """Sorts the markers by distance in ascending order"""
        self.marker_distances.sort(key=lambda x: x['distance'])

    def get_distances_list(self):
        """Returns the list of markers and their distances"""
        return self.marker_distances

    def publish_contour_markers_sorted(self):
        marker_array = MarkerArray()

        for x in self.marker_distances:
            marker = x['original_marker']
            marker_array.markers.append(marker)

        # Publish the marker array
        self.marker_sorted_publisher.publish(marker_array)

    def publish_contour_markers(self, contour_centers, map_info):
        marker_array = MarkerArray()
        resolution = map_info.resolution  # The map resolution (meters per pixel)
        origin_x = map_info.origin.position.x  # Map origin (bottom-left corner) in world coordinates
        origin_y = map_info.origin.position.y

        for idx, (pixel_x, pixel_y) in enumerate(contour_centers):
            # Convert from pixel coordinates to world coordinates
            world_x = (pixel_x * resolution) + origin_x
            world_y = (pixel_y * resolution) + origin_y

            # Create a marker for each contour center
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "contour_centers"
            marker.id = idx
            marker.type = Marker.SPHERE  # You can use SPHERE or POINT type
            marker.action = Marker.ADD

            # Set the position of the marker in the world frame
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.0  # Flat on the map, adjust if 3D is needed

            # Set the size of the marker
            marker.scale.x = 0.2  # Adjust as needed
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            # Set the color of the marker (RGBA)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully visible

            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()  # Marker is persistent

            marker_array.markers.append(marker)

        # Publish the marker array
        self.marker_publisher.publish(marker_array)


    def publish_additional_map(self, map_image, map_info, map_header):
        # Convert the grayscale map image back to an OccupancyGrid
        additional_map_msg = OccupancyGrid()
        additional_map_msg.header = map_header
        additional_map_msg.info = map_info
        
        # Since `map_image` contains 0 (free), 127 (unknown), and 255 (occupied),
        # we need to remap these values to match the OccupancyGrid format (-1, 0, 100)
        map_data = np.where(map_image == 127, -1, map_image)  # Unknown becomes -1
        map_data = np.where(map_image == 0, 0, map_data)      # Free stays 0
        map_data = np.where(map_image == 255, 100, map_data)  # Occupied becomes 100

        # Flatten and convert to list
        additional_map_msg.data = map_data.flatten().astype(np.int8).tolist()

        # Publish the additional map
        self.additional_map_publisher.publish(additional_map_msg)

    def process_map(self, map_msg):
        # Get map metadata
        width = map_msg.info.width
        height = map_msg.info.height
        map_data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # Create an empty map initialized with default grayscale values
        empty_map = np.full((height, width), 50, dtype=np.uint8)  # Default grayscale value

        # Convert the map to 8-bit grayscale image with specified values
        map_image = np.where(map_data == -1, 50, empty_map)  # -1 becomes 50 (gray for unknown)
        map_image = np.where(map_data == 0, 20, map_image)    # 0 becomes 20 (black for free)
        map_image = np.where(map_data > 0, 100, map_image)    # >0 becomes 100 (white for occupied)

        # Publish `map_image` as an additional map
        self.publish_additional_map(map_image, map_msg.info, map_msg.header)

        # Create a transition map for detecting edges from 20 to 50
        transition_map = np.zeros_like(map_image, dtype=np.uint8)

        # Check horizontal transitions
        horizontal_transition = (map_image[:, :-1] == 20) & (map_image[:, 1:] == 50)
        transition_map[:, 1:] |= horizontal_transition

        # Check vertical transitions
        vertical_transition = (map_image[:-1, :] == 20) & (map_image[1:, :] == 50)
        transition_map[1:, :] |= vertical_transition

        # Find contours from the transition map
        contours, _ = cv2.findContours(transition_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # contour_centers = []  # Store the centers of valid contours
        # for contour in contours:
        #     length = cv2.arcLength(contour, closed=False)
        #     if length > 20:  # Filter out contours shorter than 4 pixels
        #         M = cv2.moments(contour)
        #         if M['m00'] != 0:  # Avoid division by zero
        #             center_x = int(M['m10'] / M['m00'])
        #             center_y = int(M['m01'] / M['m00'])
        #             contour_centers.append((center_x, center_y))
        contour_midpoints = []  # Store the midpoints of valid contours

        for contour in contours:
            length = cv2.arcLength(contour, closed=False)
            if length > 20:  # Filter out contours shorter than 4 pixels
                # Calculate the contour midpoint
                contour_points = np.squeeze(contour)  # Remove redundant dimensions from contour array
                if len(contour_points) > 1:  # Check if contour has more than 1 point
                    midpoint_x = np.mean(contour_points[:, 0])
                    midpoint_y = np.mean(contour_points[:, 1])
                    contour_midpoints.append((midpoint_x, midpoint_y))

        self.publish_contour_markers(contour_midpoints, map_msg.info)
        # Log the detected contour centers
        # self.get_logger().info(f"Detected {len(contour_centers)} contour centers: {contour_centers}")

        # Publish the markers
        # self.publish_contour_markers(contour_centers, map_msg.info)
        



def main(args=None):
    rclpy.init(args=args)

    map_processor = MapProcessor()

    rclpy.spin(map_processor)

    map_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
