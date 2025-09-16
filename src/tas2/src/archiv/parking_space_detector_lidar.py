#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ParkingSpaceDetector(Node):
    def __init__(self):
        super().__init__('parking_space_detector')
        
        # Adjustable parameters
        self.car_length = 0.7  # Car length in meters
        self.car_width = 0.45  # Car width in meters
        self.required_space_length = 1.0  # Required parking space length
        self.required_space_width = 0.55  # Required parking space width
        self.buffer = 0.0  # Extra clearance in meters

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match QoS of the /scan topic
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/parking_spaces',
            10
        )

        self.get_logger().info("Parking Space Detector Node Initialized.")

    def lidar_callback(self, msg: LaserScan):
        # Extract right-side LIDAR data
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        # Right side: Adjust the range based on angle assumption
        start_angle = math.pi/2 # -90 degrees
        end_angle = 0.0 # 0 degrees

        # Find the indices corresponding to the angles
        start_index = int((start_angle - angle_min) / angle_increment)
        end_index = int((end_angle - angle_min) / angle_increment)

        # Extract ranges for the right side
        right_side_ranges = ranges[start_index:end_index]

        # Detect gaps large enough for parking
        gaps = self.detect_gaps(right_side_ranges, angle_increment)

        # Visualize gaps in RViz
        self.publish_markers(gaps, msg)

    def detect_gaps(self, ranges, angle_increment):
        """
        Detects gaps in the LIDAR ranges that meet parking requirements.

        Args:
            ranges (list): List of distances from the LIDAR.
            angle_increment (float): Angle between LIDAR readings.

        Returns:
            list: A list of gaps [(start_index, end_index, gap_length)].
        """
        gaps = []
        gap_start = None
        for i, distance in enumerate(ranges):
            if distance > self.required_space_width:
                if gap_start is None:
                    gap_start = i
            else:
                if gap_start is not None:
                    gap_end = i
                    gap_length = (gap_end - gap_start) * angle_increment * distance
                    if gap_length >= self.required_space_length + self.buffer:
                        gaps.append((gap_start, gap_end, gap_length))
                    gap_start = None

        # Handle the last gap if it ends at the last range
        if gap_start is not None:
            gap_end = len(ranges)
            gap_length = (gap_end - gap_start) * angle_increment * ranges[-1]
            if gap_length >= self.required_space_length + self.buffer:
                gaps.append((gap_start, gap_end, gap_length))

        return gaps

    def publish_markers(self, gaps, scan_msg):
        """
        Publishes detected gaps as markers to RViz.

        Args:
            gaps (list): List of gaps [(start_index, end_index, gap_length)].
            scan_msg (LaserScan): The original LaserScan message.
        """
        marker_array = MarkerArray()
        for i, (start_index, end_index, gap_length) in enumerate(gaps):
            start_angle = scan_msg.angle_min + start_index * scan_msg.angle_increment
            end_angle = scan_msg.angle_min + end_index * scan_msg.angle_increment

            # Compute start and end points of the gap in Cartesian coordinates
            start_distance = scan_msg.ranges[start_index]
            end_distance = scan_msg.ranges[end_index]

            start_x = start_distance * math.cos(start_angle)
            start_y = start_distance * math.sin(start_angle)
            end_x = end_distance * math.cos(end_angle)
            end_y = end_distance * math.sin(end_angle)

            # Create a marker for the gap
            marker = Marker()
            marker.header.frame_id = scan_msg.header.frame_id
            marker.header.stamp = scan_msg.header.stamp
            marker.ns = "parking_spaces"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Line width
            marker.color.a = 1.0  # Fully opaque
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green line
            marker.color.b = 0.0

            # Define the line strip points
            marker.points = [
                self.create_point(start_x, start_y),
                self.create_point(end_x, end_y)
            ]

            marker_array.markers.append(marker)

        # Publish the markers
        self.marker_publisher.publish(marker_array)

    def create_point(self, x, y):
        """
        Creates a point for a marker.
        """
        from geometry_msgs.msg import Point
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        return point


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpaceDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
