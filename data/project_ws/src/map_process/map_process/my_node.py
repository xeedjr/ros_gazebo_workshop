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
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, Point, Quaternion

class MapProcessor(Node):

    def __init__(self):
        super().__init__('map_processor_node')

        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.process_map, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'processed_map', 10)
        self.bridge = CvBridge()
        self.marker_publisher = self.create_publisher(MarkerArray, 'contour_markers', 10)
        self.additional_map_publisher = self.create_publisher(OccupancyGrid, 'additional_map', 10)


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

            marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()  # Marker is persistent

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

    def grayscale_to_raw_occupancy_grid(self, grayscale_image, original_map):
        # Extract width and height from the original map info
        original_info = original_map.info
        width = original_info.width
        height = original_info.height
        original_resolution = original_info.resolution

        # Create an OccupancyGrid message
        occupancy_grid_msg = OccupancyGrid()

        # Set up the header
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.frame_id = original_map.header.frame_id

        # Set the map metadata (resolution, width, height)
        occupancy_grid_msg.info.resolution = original_resolution  # meters per cell
        occupancy_grid_msg.info.width = width
        occupancy_grid_msg.info.height = height


        # Set the origin of the map (centered)
        occupancy_grid_msg.info.origin = original_map.info.origin

        # Flatten the grayscale image to a 1D array
        flat_image = grayscale_image.flatten()

        # Normalize grayscale values to fit in occupancy grid range [0, 100]
        # Map grayscale values [0, 255] to occupancy values [0, 100]
        occupancy_data = np.clip((flat_image / 255.0) * 100, 0, 100).astype(np.uint8)

        # Assign raw data to the OccupancyGrid message
        occupancy_grid_msg.data = occupancy_data.tolist()

        return occupancy_grid_msg

    def publish_raw_map(self, grayscale_image, map_msg):
        # Convert the grayscale image to an OccupancyGrid message
        occupancy_grid_msg = self.grayscale_to_raw_occupancy_grid(grayscale_image, map_msg)
        # Publish the OccupancyGrid
        self.additional_map_publisher.publish(occupancy_grid_msg)
        self.get_logger().info('Published raw map')

    def show_grayscale_map_non_blocking(self, grayscale_image):
        # Create a named window with the ability to resize
        cv2.namedWindow('Grayscale Map', cv2.WINDOW_NORMAL)
        # Display the grayscale image in a non-blocking manner
        cv2.imshow('Grayscale Map', grayscale_image)
        # Use a short delay to avoid blocking the execution
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to close the window
            cv2.destroyAllWindows()

    def map_to_grayscale_image(self, occupancy_grid_msg: OccupancyGrid):
        # Extract the width, height, and occupancy data
        width = occupancy_grid_msg.info.width
        height = occupancy_grid_msg.info.height
        occupancy_data = np.array(occupancy_grid_msg.data).reshape((height, width))

        # Create an empty grayscale image
        grayscale_image = np.zeros((height, width), dtype=np.uint8)

        # Assign brightness values
        # -1 or 255 (unmapped): set to 20
        # 0 (free space): set to 100
        # 100 (occupied space): set to 150
        grayscale_image[occupancy_data == -1] = 20      # Unmapped area
        grayscale_image[occupancy_data == 0] = 100      # Free space
        grayscale_image[occupancy_data > 0] = 150       # Occupied space

        return grayscale_image

    def find_pixels_with_brightness(self, image):
        """
        Finds pixels in a grayscale image with a brightness of 50 that are 
        surrounded by at least one pixel with brightness 20, and returns an 
        output image with the same size, where those pixels are marked with 
        brightness 255.
        
        Args:
            image (numpy.ndarray): Input grayscale image.
        
        Returns:
            numpy.ndarray: Output image with found pixels set to 255, others to 0.
        """
        # Create an empty output image filled with zeros (black)
        output_image = np.zeros_like(image, dtype=np.uint8)

        # Find all pixels with brightness 20
        target_pixels = np.where(image == 20)

        # Iterate through the target pixels
        for y, x in zip(*target_pixels):
            # Extract a 3x3 region around the pixel (ensure we stay within image bounds)
            roi = image[max(0, y-1):y+2, max(0, x-1):x+2]

            # Check if any pixel in the region has brightness 100
            if np.any(roi == 100):
                # Set the corresponding pixel in the output image to 100
                output_image[y, x] = 100

        return output_image

    def find_non_linear_lines_with_centers(self, image):
        """
        Find non-linear lines with length greater than 11 pixels in a grayscale image
        and return their pixel coordinates along with the center pixel of each line.
        
        Parameters:
        - image: A grayscale image where lines are marked with brightness 100.
        
        Returns:
        - A list of tuples, each containing:
            - A list of pixel coordinates that form the line.
            - The center pixel of the line as (x, y).
        """
        # Convert image to binary
        _, binary = cv2.threshold(image, 99, 255, cv2.THRESH_BINARY)
        
        # Find contours in the binary image
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        lines_with_centers = []
        
        for contour in contours:
            # Check if contour is a line (length > 11)
            if cv2.arcLength(contour, True) > 11:
                # Get the pixel coordinates
                line_pixels = [tuple(point[0]) for point in contour]
                
                # Calculate the center pixel of the line
                if line_pixels:
                    center_x = int(np.mean([p[0] for p in line_pixels]))
                    center_y = int(np.mean([p[1] for p in line_pixels]))
                    center_pixel = (center_x, center_y)
                else:
                    center_pixel = (0, 0)
                
                # Append to result
                lines_with_centers.append((line_pixels, center_pixel))
        
        return lines_with_centers

    def create_centered_image(self, image, lines):
        # Create a zeroed image with the same dimensions as the input image
        centered_image = np.zeros_like(image, dtype=np.uint8)

        # Draw the centers with brightness 120
        center_brightness = 120
        for line in lines:
            # Compute the center of the line
            line_array = np.array(line)
            center = np.mean(line_array, axis=0).astype(int)
            centered_image[center[0], center[1]] = center_brightness

        return centered_image

    def paint_centers_on_blank_image(self, image, lines_with_centers):
        """
        Create a zeroed image with the same size as the input image and paint white pixels
        at the coordinates of the line centers.
        
        Parameters:
        - image: A grayscale image used to determine the size of the zeroed image.
        - lines_with_centers: A list of tuples, where each tuple contains:
            - A list of pixel coordinates that form the line (not used in this function).
            - The center of the line as (x, y).
        
        Returns:
        - A zeroed image with white pixels painted at the line centers.
        """
        # Create a zeroed image with the same size as the input image
        blank_image = np.zeros_like(image)
        
        # Paint white pixels at the line centers
        for _, center in lines_with_centers:
            center_x, center_y = center
            if 0 <= center_x < blank_image.shape[1] and 0 <= center_y < blank_image.shape[0]:
                blank_image[center_y, center_x] = 255  # Set pixel to white (255)
        
        return blank_image

    def process_map(self, map_msg):
        map_img = self.map_to_grayscale_image(map_msg)
        
        map_img = self.find_pixels_with_brightness(map_img)

        self.show_grayscale_map_non_blocking(map_img)

        lines = self.find_non_linear_lines_with_centers(map_img)
        self.get_logger().info('Lines' + str(lines))

        # Create image with painted centers
        result_image = self.paint_centers_on_blank_image(map_img, lines)
        
        self.show_grayscale_map_non_blocking(result_image)

        self.publish_raw_map(result_image, map_msg)
        # self.show_grayscale_map_non_blocking(map_img)


def main(args=None):
    rclpy.init(args=args)

    map_processor = MapProcessor()

    rclpy.spin(map_processor)

    map_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
