#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray, TransformStamped
from std_msgs.msg import Bool
from custom_msgs.msg import ParkingStarting, StateControl  # Assuming this custom message contains the required fields
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException
from std_msgs.msg import Float32
import math
from robot_utils import RobotUtilities # fct with robot position
from nav_msgs.msg import Path

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
        self.get_logger().info("Reverse parking is disabled.")
        self.corner_0 = None
        self.corner_1 = None
        self.corner_2 = None
        self.corner_3 = None
        self.robot_pos = None
        self.car_length=0.65
        self.car_width=0.45

        # to get robot position
        self.utilities = RobotUtilities(self)

        self.latest_wall_distance = None
        self.parking_length = None
        self.radius=None
        self.start_x = None
        self.start_y = None

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a publisher for the /goal_pose topic
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Publisher for path visualization
        self.path_publisher_ = self.create_publisher(Path, '/planned_path', 10)

        # self.parking_sub = self.create_subscription(
        #     ParkingStarting, '/parking_starting', self.start_parking_callback, 10)

        self.rev_parking_status = self.create_subscription(
            StateControl, '/state_control_topic', self.start_parking_callback, 10)
        

        # Subscribe to the 'wall_distance' topic
        self.subscription = self.create_subscription(
            Float32,
            'wall_distance',  
            self.wall_distance_callback,
            10  
        )
        self.length_sub = self.create_subscription(
            Float32,
            '/parking_gap_length', 
            self.parking_length_callback,
            10  
        )
        self.subscription 
        
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

    def wall_distance_callback(self, msg):
        """
        Callback function for the wall_distance topic.
        """
        # Store the latest message
        self.latest_wall_distance = msg.data
        self.get_logger().info(f"Updated latest wall distance: {self.latest_wall_distance:.2f} meters")
    
    def parking_length_callback(self, msg):
        """
        Callback function for the parking length topic.
        """
        # Store the latest message
        self.parking_length = msg.data
        self.get_logger().info(f"Updated parking length: {self.parking_length:.2f} meters")


    def process_closest_corner(self):
        """
        Main processing loop to find and publish the closest adjusted corner.
        """
        # get robot position, return if not available    
        current_pos = self.utilities.get_robot_position()
        if not current_pos:
            return
        self.start_x,self.start_y = current_pos
        # self.start_x=robot_pos.x
        # self.start_y=robot_pos.y

        # Find the closest corner and adjust its position
        # adjusted_x, adjusted_y = self.find_closest_corner_and_adjust()

        # return adjusted_x,adjusted_y
        # return adjusted_x,adjusted_y

    def start_parking_callback(self, msg):
        """
        Callback to handle parameters from the parking_starting topic.
        """
        self.get_logger().info("Received a message on /parking_starting")
        self.rev_parking = msg.rev_parking
        if not self.rev_parking:
            self.get_logger().info("Reverse parking is disabled. Ignoring the message.")
            return

        if self.start_x is None or self.start_y is None:
        # Find the closest corner only once
            self.start_x, self.start_y = self.utilities.get_robot_position()
            if self.start_x is None or self.start_y is None:
                self.get_logger().warn("Failed to calculate start position. Aborting.")
                return

        self.get_logger().info("Received parking parameters. Generating trajectory...")

        
        distance_to_back_wall_of_gap = 0.3
        start_heading = 0.0
        backward_distance = 0.0
        self.radius=self.latest_wall_distance+self.car_width
        radius_left = self.radius-self.car_width/2
        radius_right = self.radius+self.car_width/2
        forward_distance = 0.0
        arc_angle_left = np.rad2deg(np.arctan2(self.parking_length - distance_to_back_wall_of_gap,self.radius))
        arc_angle_right = np.rad2deg(np.arctan2(self.parking_length - distance_to_back_wall_of_gap,self.radius))
        step_size = 0.05

        # Generate the trajectory
        self.trajectory = generate_reverse_parking_trajectory(
            self.start_x, self.start_y, start_heading,
            backward_distance, radius_left, radius_right,
            forward_distance, arc_angle_left, arc_angle_right,
            step_size
        )
        self.current_index = 0
        # Publish the planned path for RViz visualization
        # self.publish_path()

        # Start publishing the trajectory
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(0.2, self.publish_point)

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