#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from math import atan2, sqrt, pi


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 1)
        self.create_subscription(PoseStamped, '/trajectory', self.trajectory_callback, 1)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Variables for current pose and trajectory
        self.current_pose = None
        self.trajectory = []
        self.current_target_index = 0

        # Error accumulation
        self.error_x = 0.0
        self.error_y = 0.0

        # PID controller gains (adjusted for backward motion)
        self.kp_linear = 1.5  # Linear gain for backward velocity
        self.kp_angular = 2.0  # Angular gain
        self.max_reverse_velocity = -0.3  # Max reverse linear velocity
        self.max_angular_velocity = 1.0  # Max angular velocity

        # Tolerance for waypoint reaching (not important for switching to next waypoint at 5 Hz)
        self.waypoint_tolerance = 0.1  # meters

        # Timer for control loop (5 Hz for trajectory update, 10 Hz for velocity publishing)
        self.timer_trajectory = self.create_timer(0.2, self.update_trajectory)  # 5 Hz
        self.timer_control = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def amcl_pose_callback(self, msg):
        """Callback to get the current robot pose from AMCL."""
        self.current_pose = msg.pose.pose
        self.get_logger().info(
            f"Current Pose: x={self.current_pose.position.x:.2f}, y={self.current_pose.position.y:.2f}"
        )

    def trajectory_callback(self, msg):
        """Callback to get a single trajectory waypoint."""
        self.trajectory.append(msg)
        self.get_logger().info(f"Waypoint added: x={msg.pose.position.x}, y={msg.pose.position.y}")

    def update_trajectory(self):
        """Update the trajectory to the next point every 5 Hz."""
        if self.current_target_index < len(self.trajectory):
            # Update to the next target every time, regardless of distance.
            self.current_target_index += 1
            self.get_logger().info(f"Moving to waypoint {self.current_target_index + 1}")

    def control_loop(self):
        """Main loop to process trajectory and current pose at 10 Hz."""
        if self.current_pose is None or not self.trajectory or self.current_target_index >= len(self.trajectory):
            return

        # Get the current target waypoint
        target_pose = self.trajectory[self.current_target_index].pose
        target_x = target_pose.position.x
        target_y = target_pose.position.y

        # Calculate distance and angle to the target waypoint
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        distance = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)

        # Extract current heading from quaternion
        current_orientation = self.current_pose.orientation
        current_yaw = self.quaternion_to_yaw(
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w,
        )

        # Calculate angular difference
        angle_diff = self.normalize_angle(angle_to_goal - current_yaw)

        # Dynamic backward linear velocity based on the error (accumulated errors)
        self.error_x += dx
        self.error_y += dy
        linear_velocity = max(self.kp_linear * -distance, self.max_reverse_velocity)

        # Compute angular velocity to steer towards the target waypoint
        angular_velocity = max(-self.max_angular_velocity, min(self.kp_angular * angle_diff, self.max_angular_velocity))

        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel)

        # Log movement
        self.get_logger().info(f"Target: x={target_x:.2f}, y={target_y:.2f}, "
                               f"Distance: {distance:.2f}, Angle Diff: {angle_diff:.2f}, "
                               f"Linear Vel: {linear_velocity:.2f}, Angular Vel: {angular_velocity:.2f}")

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle (in radians)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
