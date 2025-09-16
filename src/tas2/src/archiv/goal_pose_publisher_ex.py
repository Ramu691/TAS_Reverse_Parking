#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from custom_msgs.msg import ParkingStarting  # Assuming this custom message contains the required fields
from tf_transformations import quaternion_from_euler
import math

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

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.
    :param roll: Rotation around the X-axis in radians.
    :param pitch: Rotation around the Y-axis in radians.
    :param yaw: Rotation around the Z-axis in radians.
    :return: Quaternion as a tuple (x, y, z, w).
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return qx, qy, qz, qw

def yaw_to_quaternion(yaw):
    """
    Convert a yaw angle (in radians) to a quaternion.
    :param yaw: Rotation around the Z-axis in radians.
    :return: Quaternion as a tuple (x, y, z, w).
    """
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return qx, qy, qz, qw


class MultiGoalPosePublisher(Node):
    def __init__(self):
        super().__init__('multi_goal_pose_publisher')
        self.get_logger().info("Reverse parking is disabled. Ignoring the message.")

        # Create a publisher for the /goal_pose topic
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.parking_sub = self.create_subscription(
            ParkingStarting, '/parking_starting', self.start_parking_callback, 10)

        # List of goal poses (x, y, yaw in radians)
        # self.goal_poses = [
        #     (-10.5, 10.0, 90.0),
        #     (-11.5, 8.0, 30.0),
        #     (-8.42,6.2,0.0)
        # ]
        # Timer variables
        self.timer = None
        self.trajectory = []
        self.current_index = 0
        # Reverse parking flag
        self.rev_parking = False
        # Index to track the current goal pose
        #self.current_goal_index = 0

        # Timer to publish goal poses sequentially
        #self.timer = self.create_timer(15.0, self.publish_next_goal_pose)
    
    def start_parking_callback(self, msg):
        """
        Callback to handle parameters from the parking_starting topic.
        """
        self.get_logger().info("Received a message on /parking_starting")
        self.rev_parking = msg.rev_parking
        if not self.rev_parking:
            self.get_logger().info("Reverse parking is disabled. Ignoring the message.")
            return

        self.get_logger().info("Received parking parameters. Generating trajectory...")

        # Extract parameters from the message
        start_x = msg.start_x
        start_y = msg.start_y
        start_heading = msg.start_heading
        backward_distance = msg.backward_distance - 0.5
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
        self.timer = self.create_timer(0.3, self.publish_point)

    def publish_point(self):
        """
        Publishes a single trajectory point as a PoseStamped message.
        """
        if self.current_index < len(self.trajectory):
            x, y, heading = self.trajectory[self.current_index]

            # Convert heading to quaternion
            quaternion = quaternion_from_euler(0.0, 0.0, heading)

            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            #pose_msg.header.frame_id = 'odom'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            self.publisher_.publish(pose_msg)

            self.get_logger().info(f"Published point {self.current_index + 1}: "
                                   f"(x={x:.2f}, y={y:.2f}, theta={heading:.2f})")

            # Increment the index
            self.current_index += 1
        else:
            self.get_logger().info("All trajectory points have been published.")
            if self.timer:
                self.timer.cancel()

    # def publish_next_goal_pose(self):
    #     if self.current_goal_index < len(self.goal_poses):
    #         x, y, yaw = self.goal_poses[self.current_goal_index]
    #         #qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
    #         #x, y, yaw = self.goal_poses[self.current_goal_index]
    #         qx, qy, qz, qw = yaw_to_quaternion(yaw)

    #         # Create the PoseStamped message
    #         msg = PoseStamped()
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.header.frame_id = 'map'

    #         msg.pose.position.x = x
    #         msg.pose.position.y = y
    #         msg.pose.position.z = 0.0

    #         msg.pose.orientation.x = qx
    #         msg.pose.orientation.y = qy
    #         msg.pose.orientation.z = qz
    #         msg.pose.orientation.w = qw

    #         # Publish the message
    #         self.publisher_.publish(msg)
    #         self.get_logger().info(f'Published goal pose {self.current_goal_index + 1}: Position ({x}, {y}, {0.0}), Orientation ({qx}, {qy}, {qz}, {qw})')

    #         # Move to the next goal pose
    #         self.current_goal_index += 1
    #     else:
    #         self.get_logger().info('All goal poses have been published.')
    #         self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# import math


# def euler_to_quaternion(roll, pitch, yaw):
#     """
#     Convert Euler angles (roll, pitch, yaw) to a quaternion.
#     :param roll: Rotation around the X-axis in radians.
#     :param pitch: Rotation around the Y-axis in radians.
#     :param yaw: Rotation around the Z-axis in radians.
#     :return: Quaternion as a tuple (x, y, z, w).
#     """
#     qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
#     qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
#     qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
#     qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
#     return qx, qy, qz, qw

# def yaw_to_quaternion(yaw):
#     """
#     Convert a yaw angle (in radians) to a quaternion.
#     :param yaw: Rotation around the Z-axis in radians.
#     :return: Quaternion as a tuple (x, y, z, w).
#     """
#     qx = 0.0
#     qy = 0.0
#     qz = math.sin(yaw / 2)
#     qw = math.cos(yaw / 2)
#     return qx, qy, qz, qw


# class MultiGoalPosePublisher(Node):
#     def __init__(self):
#         super().__init__('multi_goal_pose_publisher')

#         # Create a publisher for the /goal_pose topic
#         self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

#         # List of goal poses (x, y, yaw in radians)
#         self.goal_poses = [
#             (-10.5, 10.0, 90.0),
#             (-11.5, 8.0, 30.0),
#             (-8.42,6.2,0.0)
#         ]

#         # Index to track the current goal pose
#         self.current_goal_index = 0

#         # Timer to publish goal poses sequentially
#         self.timer = self.create_timer(15.0, self.publish_next_goal_pose)

#     def publish_next_goal_pose(self):
#         if self.current_goal_index < len(self.goal_poses):
#             x, y, yaw = self.goal_poses[self.current_goal_index]
#             #qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
#             #x, y, yaw = self.goal_poses[self.current_goal_index]
#             qx, qy, qz, qw = yaw_to_quaternion(yaw)

#             # Create the PoseStamped message
#             msg = PoseStamped()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.header.frame_id = 'map'

#             msg.pose.position.x = x
#             msg.pose.position.y = y
#             msg.pose.position.z = 0.0

#             msg.pose.orientation.x = qx
#             msg.pose.orientation.y = qy
#             msg.pose.orientation.z = qz
#             msg.pose.orientation.w = qw

#             # Publish the message
#             self.publisher_.publish(msg)
#             self.get_logger().info(f'Published goal pose {self.current_goal_index + 1}: Position ({x}, {y}, {0.0}), Orientation ({qx}, {qy}, {qz}, {qw})')

#             # Move to the next goal pose
#             self.current_goal_index += 1
#         else:
#             self.get_logger().info('All goal poses have been published.')
#             self.timer.cancel()


# def main(args=None):
#     rclpy.init(args=args)
#     node = MultiGoalPosePublisher()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# import math


# def euler_to_quaternion(roll, pitch, yaw):
#     """
#     Convert Euler angles (roll, pitch, yaw) to a quaternion.
#     :param roll: Rotation around the X-axis in radians.
#     :param pitch: Rotation around the Y-axis in radians.
#     :param yaw: Rotation around the Z-axis in radians.
#     :return: Quaternion as a tuple (x, y, z, w).
#     """
#     qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
#     qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
#     qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
#     qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
#     return qx, qy, qz, qw


# class GoalPosePublisher(Node):
#     def __init__(self):
#         super().__init__('goal_pose_publisher')

#         # Create a publisher for the /goal_pose topic
#         self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

#         # Publish a point after initialization
#         self.timer = self.create_timer(1.0, self.publish_goal_pose)

#     def publish_goal_pose(self):
#         # Define the goal position
#         x = -10.519062042236328
#         y = 7.373376846313477
#         z = 0.0

#         # Define the orientation in radians (yaw only)
#         yaw = -0.6  # Example yaw in radians
#         roll = 0.0
#         pitch = 0.0

#         # Convert the orientation to a quaternion
#         qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

#         # Create the PoseStamped message
#         msg = PoseStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'map'

#         msg.pose.position.x = x
#         msg.pose.position.y = y
#         msg.pose.position.z = z

#         msg.pose.orientation.x = qx
#         msg.pose.orientation.y = qy
#         msg.pose.orientation.z = qz
#         msg.pose.orientation.w = qw

#         # Publish the message
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Published goal pose: Position ({x}, {y}, {z}), Orientation ({qx}, {qy}, {qz}, {qw})')


# def main(args=None):
#     rclpy.init(args=args)
#     node = GoalPosePublisher()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
