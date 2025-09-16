#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import time

# PPS = Potential parking space
class WallFollowerWithEnhancedPPS(Node):
    def __init__(self):
        super().__init__('wall_follower_with_enhanced_pps')

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match QoS of the /scan topic
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # LiDAR subscription
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        # Velocity publisher
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        ####################### need to adapt this to our needs based on testing in lab on real vehicle
        # Parameters
        self.target_distance_to_wall = 0.5  # Desired distance from the wall (meters)
        self.distance_threshold = 0.5  # Threshold for detecting gaps (meters)
        self.state = "FOLLOW_WALL"  # Initial state
        self.target_position = None  # Target position to navigate past PPS

        # PID parameters
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.5  # Derivative gain
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        #######################

    def follow_wall(self, ranges, range_min, range_max):
        """Wall-following behavior using a PID controller."""
        # Extract the closest distance to the wall on the right
        valid_ranges = ranges[(ranges >= range_min) & (ranges <= range_max)]
        if len(valid_ranges) == 0:
            self.get_logger().warn("No valid LiDAR data for wall-following!")
            return

        closest_distance = np.min(valid_ranges)

        # Compute the error
        error = self.target_distance_to_wall - closest_distance

        # PID calculations
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        proportional = self.kp * error
        self.integral += error * delta_time
        integral = self.ki * self.integral
        derivative = self.kd * (error - self.previous_error) / delta_time
        self.previous_error = error

        # Calculate angular correction
        angular_correction = proportional + integral + derivative

        # Publish the Twist command for wall-following
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Constant forward speed
        twist_msg.angular.z = angular_correction
        self.publisher.publish(twist_msg)

        self.get_logger().info(
            f"Wall-following: Distance: {closest_distance:.2f}, Error: {error:.2f}, "
            f"P: {proportional:.2f}, I: {integral:.2f}, D: {derivative:.2f}, Angular Z: {angular_correction:.2f}"
        )

    def lidar_callback(self, msg):
    
        # Convert ranges to a numpy array
        ranges = np.array(msg.ranges)

        # Filter out invalid data (inf or NaN)
        valid_ranges = ranges[np.isfinite(ranges)]

        # Store filtered ranges for further processing
        range_min = msg.range_min
        range_max = msg.range_max
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        # Extract right-side LiDAR data (~120° arc)
        start_index = int((math.radians(-90) - angle_min) / angle_increment)
        end_index = int((math.radians(-30) - angle_min) / angle_increment)
        right_side_ranges = valid_ranges[start_index:end_index]

        # Handle state-specific behaviors
        if self.state == "FOLLOW_WALL":
            self.follow_wall(right_side_ranges, range_min, range_max)
            self.detect_parking_space(valid_ranges, angle_min, angle_increment, range_min, range_max)
        elif self.state == "NAVIGATE_TO_TARGET":
            self.navigate_to_target()

    def detect_parking_space(self, ranges, angle_min, angle_increment, range_min, range_max):
        """Detect and validate a potential parking space."""
        gap_start = None
        gap_end = None

        # Extend the angle range
        start_index = int((math.radians(-120) - angle_min) / angle_increment)
        end_index = int((math.radians(0) - angle_min) / angle_increment)
        relevant_ranges = ranges[start_index:end_index]

        # Filter valid ranges
        relevant_ranges = relevant_ranges[np.isfinite(relevant_ranges)]
        if len(relevant_ranges) == 0:
            self.get_logger().warn("No valid LiDAR data for gap detection!")
            return

        # Detect start and end of the PPS
        for i in range(len(relevant_ranges) - 1):
            if range_min <= relevant_ranges[i] <= range_max:
                if relevant_ranges[i + 1] - relevant_ranges[i] > self.distance_threshold:
                    gap_start = i
                    self.get_logger().info(f"Gap start detected at {relevant_ranges[gap_start]:.2f}m")
                elif gap_start is not None and relevant_ranges[i] - relevant_ranges[i + 1] > self.distance_threshold:
                    gap_end = i
                    self.get_logger().info(f"Gap end detected at {relevant_ranges[gap_end]:.2f}m")
                    break

        # If gap start and end are detected, calculate navigation target
        #################
        # tbd:
        # add, that if gap exceeds certain length, it is not a parking place but something else (street etc) --> differentiate here
        #################
        if gap_start is not None and gap_end is not None:
            gap_width = relevant_ranges[gap_end] - relevant_ranges[gap_start]
            self.get_logger().info(f"Gap detected: Start={relevant_ranges[gap_start]:.2f}m, End={relevant_ranges[gap_end]:.2f}m, Width={gap_width:.2f}m")
            self.calculate_target_position(relevant_ranges[gap_end])
            self.state = "NAVIGATE_TO_TARGET"

    def calculate_target_position(self, gap_end_distance):
        """Calculate a navigation target past the end of the PPS."""
        self.target_position = gap_end_distance - self.target_distance_to_wall
        self.get_logger().info(f"Target position set to {self.target_position:.2f}m")

    def navigate_to_target(self):
        """Navigate directly to the target position past the PPS."""
        if self.target_position is not None:
            twist_msg = Twist()
            twist_msg.linear.x = 0.2  # Move forward
            twist_msg.angular.z = 0.0  # Go straight

            #################
            # add here: store current position in the map and pos of gap (geometry of parking space)--> use this for parking manouveour
            #################
            # Use LiDAR data to determine when to stop
            current_distance = self.get_distance_to_target()
            if current_distance is not None and current_distance <= self.target_position:
                twist_msg.linear.x = 0.0  # Stop
                self.target_position = None  # Reset target
                self.state = "FOLLOW_WALL"  # Return to wall-following
                self.get_logger().info("Reached target position. Returning to wall-following.")

            self.publisher.publish(twist_msg)

    def get_distance_to_target(self):
        """Use LiDAR data to get the distance to the target position."""
        return self.target_position


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerWithEnhancedPPS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
