#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import time

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

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
        self.distance_threshold = 0.5 # Threshold for detecting gaps (meters)
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
            self.get_logger().warn("No valid LiDAR data for wall-following! Keep driving forward.")
            return

        closest_distance = np.min(valid_ranges)

        # Compute the error
        error = self.target_distance_to_wall - closest_distance

        # threshhold 0.8 found out by testing. This prevents car from turning into gaps between cars / in the parking gap.
        if error < 0.8 and error >-0.8:
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
        else:
            self.state="DRIVING_FORWARD"
            self.get_logger().info("Detected error too large. Suppousedly corner street or parking space detected. Continue straight forward.")

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

        # Handle state-specific behaviors
        if self.state == "FOLLOW_WALL":
            # Extract right-side LiDAR data (~120° arc)
            start_index = int((math.radians(-90) - angle_min) / angle_increment)
            end_index = int((math.radians(-30) - angle_min) / angle_increment)
            right_side_ranges = valid_ranges[start_index:end_index]
            self.follow_wall(right_side_ranges, range_min, range_max)
        if self.state == "DRIVING_FORWARD": # if we decide to use this, logic could be added here to return to wall-following when 
            # useful lidar data received
            return


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()