#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32,String

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Parameters
        self.car_length = 2.0  # Meters
        self.car_width = 1.5  # Meters
        self.step_size = 0.1
        self.max_steering_angle = math.radians(45)

        self.start_pose = (8.0, -0.5)


        self.parking_start = (3.0, -1.5)  # Example parking spot start
        self.parking_length = 4.0        # Example parking length
        self.parking_width = 2.0         # Example parking width

        self.get_logger().info("Received parking parameters and start pose.")

        # Publishers
        self.trajectory_publisher = self.create_publisher(Path, '/trajectory', 10)
        self.phase_publisher = self.create_publisher(String, '/control_phase', 10)

        # # Subscribers
        # self.parking_request_subscriber = self.create_subscription(
        #     PoseStamped,
        #     """'/parking_request',"""
        #     self.parking_request_callback,
        #     10
        # )

        # self.reverse_parking_status_subscriber = self.create_subscription(
        #     Bool=True,
        #     self.reverse_parking_status_callback,
        #     10
        # )

        # State Variables
        self.parking_status = True
        # self.parking_start = None
        # self.parking_length = None
        # self.parking_width = None
        # self.start_pose = None

        self.get_logger().info("Trajectory Generator Node Started")

    # def reverse_parking_status_callback(self):
    #     """
    #     Callback for reverse parking status.
    #     If true, start reading parameters and generating the trajectory.
    #     """
    #     #self.parking_status = msg.data
    #     self.parking_status = True
        if self.parking_status and self.parking_start and self.parking_length and self.parking_width and self.start_pose:
            self.generate_and_publish_trajectory()

    #def parking_request_callback(self, msg):
    #def parking_request_callback(self):
        """
        Callback to process parking parameters and car's starting pose.
        """
        #self.start_pose = msg.pose
        # self.start_pose = (8.0, -0.5)


        # self.parking_start = (3.0, -1.5)  # Example parking spot start
        # self.parking_length = 4.0        # Example parking length
        # self.parking_width = 2.0         # Example parking width

        # self.get_logger().info("Received parking parameters and start pose.")

    def calculate_parking_middle(self):
        """
        Calculate the middle point of the parking spot dynamically.
        """
        if not self.parking_start or not self.parking_length or not self.parking_width:
            self.get_logger().warn("Parking parameters are missing!")
            return None

        start_x, start_y = self.parking_start
        middle_x = start_x + self.parking_length / 2
        middle_y = start_y - self.parking_width / 2

        return middle_x, middle_y

    def generate_and_publish_trajectory(self):
        """
        Generate and publish the reverse parking trajectory.
        """
        parking_middle = self.calculate_parking_middle()
        if not parking_middle:
            self.get_logger().warn("Cannot calculate parking middle point. Waiting for trajectory generation.")
            return

        trajectory, phases = self.generate_reverse_trajectory(
            start_pose=self.start_pose,
            parking_start=self.parking_start,
            parking_middle=parking_middle
        )

        if trajectory:
            self.publish_trajectory(trajectory)
            self.publish_phases(phases)
            self.get_logger().info("Trajectory and phases generated and published!")
        else:
            self.get_logger().warn("Failed to generate trajectory.")

    def generate_reverse_trajectory(self, start_pose, parking_start, parking_middle):
        """
        Generate a reverse trajectory dynamically with three phases:
        - Phase 1: Reverse to align with the outer edge of the parking spot.
        - Phase 2: Reverse further to the middle of the parking spot.
        - Phase 3: Move forward to center and align the car within the parking spot.
        """
        #x, y, theta = start_pose.position.x, start_pose.position.y, 0.0  # Assuming initial theta=0
        x, y, theta = start_pose.x, start_pose.y, 0.0
        trajectory = []
        phases = []

        # Phase 1: Reverse to align with the outer edge of the parking spot
        self.get_logger().info("Starting Phase 1: Aligning with the outer edge of parking spot.")
        while y > parking_start[1]:
            trajectory.append((x, y))
            phases.append("reverse")
            y -= self.step_size

        # Phase 2: Reverse to align with the middle of the parking spot
        self.get_logger().info("Starting Phase 2: Aligning with the middle of parking spot.")
        while y > parking_middle[1]:
            trajectory.append((x, y))
            phases.append("reverse")
            y -= self.step_size

        # Phase 3: Move forward to center and align with the parking spot
        self.get_logger().info("Starting Phase 3: Centering in the parking spot.")
        while x < parking_middle[0]:
            trajectory.append((x, y))
            phases.append("forward")
            x += self.step_size

        return trajectory, phases

    def publish_trajectory(self, trajectory):
        """
        Publish trajectory as a ROS Path message.
        """
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in trajectory:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0  # Simplified orientation
            path_msg.poses.append(pose)

        self.trajectory_publisher.publish(path_msg)

    def publish_phases(self, phases):
        """
        Publish phases as a sequence.
        """
        for phase in phases:
            phase_msg = String()
            phase_msg.data = phase
            self.phase_publisher.publish(phase_msg)


    # def control_law(self, x, y, theta, target_x, target_y):
    #     """
    #     Control law to calculate steering angle and speed.
    #     """
    #     dx = target_x - x
    #     dy = target_y - y
    #     desired_theta = math.atan2(dy, dx)
    #     steering_angle = desired_theta - theta
    #     steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
    #     speed = -0.1  # Reverse
    #     return steering_angle, speed

    # def kinematic_model(self, x, y, theta, steering_angle, speed, dt=0.1):
    #     """
    #     Update position using the kinematic model.
    #     """
    #     x_new = x + speed * math.cos(theta) * dt
    #     y_new = y + speed * math.sin(theta) * dt
    #     theta_new = theta + (speed / self.car_length) * math.tan(steering_angle) * dt
    #     return x_new, y_new, theta_new

    # def publish_trajectory(self, trajectory):
    #     """
    #     Publish trajectory as a ROS Path message.
    #     """
    #     path_msg = Path()
    #     path_msg.header.frame_id = 'map'
    #     path_msg.header.stamp = self.get_clock().now().to_msg()

    #     for point in trajectory:
    #         pose = PoseStamped()
    #         pose.header.frame_id = 'map'
    #         pose.header.stamp = self.get_clock().now().to_msg()
    #         pose.pose.position.x = point[0]
    #         pose.pose.position.y = point[1]
    #         pose.pose.orientation.w = 1.0  # Simplified orientation
    #         path_msg.poses.append(pose)

    #     self.trajectory_publisher.publish(path_msg)


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
