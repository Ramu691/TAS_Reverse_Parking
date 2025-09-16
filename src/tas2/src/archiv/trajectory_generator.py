#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from custom_msgs.msg import ParkingStarting 
from tf_transformations import quaternion_from_euler

def generate_reverse_parking_trajectory(
        start_x, start_y, start_heading,
        backward_distance, radius_left, radius_right,
        forward_distance, arc_angle_left, arc_angle_right, step_size):
    """
    Generates a reverse parking trajectory with specified parameters.
    """
    trajectory = []

    # Phase 1: Straight backward movement
    for _ in range(int(backward_distance / step_size)):
        start_x -= step_size * np.cos(start_heading)
        start_y -= step_size * np.sin(start_heading)
        trajectory.append((start_x, start_y, start_heading))

    # Phase 2: Reverse with a left turn (arc)
    arc_angle_left_rad = np.radians(arc_angle_left)
    for theta in np.linspace(0, arc_angle_left_rad, int(arc_angle_left_rad * radius_left / step_size)):
        dx = step_size * np.cos(start_heading + theta)
        dy = step_size * np.sin(start_heading + theta)
        start_x -= dx
        start_y -= dy
        trajectory.append((start_x, start_y, start_heading + theta))

    # Update heading for the transition to the next arc
    start_heading += arc_angle_left_rad

    # Phase 3: Reverse with a right turn (arc)
    arc_angle_right_rad = np.radians(arc_angle_right)
    for theta in np.linspace(0, -arc_angle_right_rad, int(arc_angle_right_rad * radius_right / step_size)):
        dx = step_size * np.cos(start_heading + theta)
        dy = step_size * np.sin(start_heading + theta)
        start_x -= dx
        start_y -= dy
        trajectory.append((start_x, start_y, start_heading + theta))

    # Update heading for forward movement
    start_heading -= arc_angle_right_rad

    # Phase 4: Straight forward movement (align with initial heading)
    heading_correction = (start_heading - trajectory[0][2]) % (2 * np.pi)
    if heading_correction > np.pi:
        heading_correction -= 2 * np.pi  # Normalize to [-pi, pi]

    for _ in range(int(forward_distance / step_size)):
        start_x += step_size * np.cos(start_heading - heading_correction)
        start_y += step_size * np.sin(start_heading - heading_correction)
        trajectory.append((start_x, start_y, trajectory[0][2]))  # Match the starting heading

    return trajectory

class TrajectoryGenerator(Node):
    """
    ROS 2 Node to generate and publish reverse parking trajectory points as PoseStamped.
    """

    def __init__(self):
        super().__init__('trajectory_generator')

        # Publisher
        self.trajectory_pub = self.create_publisher(PoseStamped, 'trajectory', 10)

        # Subscriber for parking_starting topic
        self.parking_sub = self.create_subscription(
            ParkingStarting, 'parking_starting', self.start_parking_callback, 10)

        # Timer variables
        self.timer = None
        self.trajectory = []
        self.current_index = 0

        # Reverse parking flag
        self.rev_parking = False

    def start_parking_callback(self, msg):
        """
        Callback to handle parameters from the parking_starting topic.
        """
        self.rev_parking = msg.rev_parking
        if not self.rev_parking:
            self.get_logger().info("Reverse parking is disabled. Ignoring the message.")
            return

        self.get_logger().info("Received parking parameters. Generating trajectory...")

        # Extract parameters from the message
        start_x = msg.start_x
        start_y = msg.start_y
        start_heading = msg.start_heading
        backward_distance = msg.backward_distance
        radius_left = msg.radius_left
        radius_right = msg.radius_right
        forward_distance = msg.forward_distance
        arc_angle_left = msg.arc_angle_left
        arc_angle_right = msg.arc_angle_right
        step_size = msg.step_size

        # Generate the trajectory
        self.trajectory = generate_reverse_parking_trajectory(
            start_x, start_y, start_heading,
            backward_distance, radius_left, radius_right,
            forward_distance, arc_angle_left, arc_angle_right,
            step_size
        )
        self.current_index = 0

        # Start publishing the trajectory
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(0.1, self.publish_point)

    def publish_point(self):
        # Check if there are points left to publish
        if self.current_index < len(self.trajectory):
            x, y, heading = self.trajectory[self.current_index]

            # Convert heading to quaternion
            quaternion = quaternion_from_euler(0.0, 0.0, heading)

            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            self.trajectory_pub.publish(pose_msg)

            self.get_logger().info(f"Published point {self.current_index + 1}: "
                                   f"(x={x:.2f}, y={y:.2f}, theta={heading:.2f})")

            # Increment the index
            self.current_index += 1
        else:
            self.get_logger().info("All trajectory points have been published.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


