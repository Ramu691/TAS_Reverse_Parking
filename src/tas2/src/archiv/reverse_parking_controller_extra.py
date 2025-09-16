import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion

class ReverseParkingController(Node):
    def __init__(self):
        super().__init__('reverse_parking_controller')
        self.get_logger().info("Reverse Parking Controller Node Started")

        # Parameters
        self.look_ahead_distance = 0.5  # Look-ahead distance for trajectory points
        self.angular_gain = 2.0        # Gain for angular velocity adjustment
        self.step_size = 0.1           # Step size for phase checking

        # Parking spot target points
        self.parking_start = (3.0, -1.5)  # Outer edge of the parking spot
        self.parking_middle = (5.0, -2.5)  # Middle of the parking spot
        self.parking_end = (5.0, -3.5)     # Center of the parking spot

        # Phase state
        self.current_phase = 1  # Start in Phase 1
        self.current_pose = None
        self.trajectory = []

        # Subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        self.trajectory_subscriber = self.create_subscription(
            Path,
            '/trajectory',
            self.trajectory_callback,
            10
        )

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def trajectory_callback(self, msg: Path):
        """Receive the trajectory from the trajectory generator."""
        self.trajectory = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info(f"Received trajectory with {len(self.trajectory)} points.")

    def pose_callback(self, msg: PoseStamped):
        """Update the robot's current pose."""
        orientation_q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def control_loop(self):
        """Main control loop to follow the trajectory and handle phase transitions."""
        if not self.trajectory:
            self.get_logger().info("No trajectory to follow.")
            return
        if not self.current_pose:
            self.get_logger().info("Waiting for current pose.")
            return

        # Update phase dynamically
        self.update_phase()

        # Select target point based on the phase
        if self.current_phase == 1:
            target_point = self.parking_start
            linear_velocity = -0.2  # Reverse

        elif self.current_phase == 2:
            target_point = self.parking_middle
            linear_velocity = -0.1  # Slower reverse

        elif self.current_phase == 3:
            target_point = self.parking_end
            linear_velocity = 0.2  # Drive forward

        else:
            self.get_logger().warn("Invalid phase. Stopping.")
            self.publish_stop_command()
            return

        # Calculate control commands
        control_command = self.calculate_control(target_point, linear_velocity)

        # Publish the control commands
        self.cmd_vel_publisher.publish(control_command)

    def update_phase(self):
        """Update the parking phase based on the robot's position."""
        if not self.current_pose:
            return

        current_x, current_y, _ = self.current_pose

        if self.current_phase == 1 and current_y <= self.parking_start[1]:
            self.current_phase = 2
            self.get_logger().info("Transitioned to Phase 2: Aligning with parking middle.")

        elif self.current_phase == 2 and current_y <= self.parking_middle[1]:
            self.current_phase = 3
            self.get_logger().info("Transitioned to Phase 3: Driving forward to center.")

    def calculate_control(self, target_point, linear_velocity):
        """Calculate linear and angular velocity for the robot to follow the target."""
        current_x, current_y, current_yaw = self.current_pose
        target_x, target_y = target_point

        # Calculate the heading error
        desired_yaw = math.atan2(target_y - current_y, target_x - current_x)
        yaw_error = desired_yaw - current_yaw

        # Normalize yaw error to the range [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # Calculate control commands
        control_command = Twist()
        control_command.linear.x = linear_velocity
        control_command.angular.z = self.angular_gain * yaw_error

        return control_command

    def publish_stop_command(self):
        """Publish a stop command to halt the robot."""
        stop_command = Twist()
        self.cmd_vel_publisher.publish(stop_command)


def main(args=None):
    rclpy.init(args=args)
    node = ReverseParkingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion

class ReverseParkingControllerWithPID(Node):
    def __init__(self):
        super().__init__('reverse_parking_controller_with_pid')
        self.get_logger().info("Reverse Parking Controller with PID Node Started")

        # Parameters
        self.look_ahead_distance = 0.5
        self.step_size = 0.1

        # PID parameters for angular control
        self.kp = 2.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.5  # Derivative gain
        self.integral_error = 0.0
        self.previous_error = 0.0

        # Parking spot target points
        self.parking_start = (3.0, -1.5)
        self.parking_middle = (5.0, -2.5)
        self.parking_end = (5.0, -3.5)

        # Phase state
        self.current_phase = 1
        self.current_pose = None
        self.trajectory = []

        # Subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        self.trajectory_subscriber = self.create_subscription(
            Path,
            '/trajectory',
            self.trajectory_callback,
            10
        )

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def trajectory_callback(self, msg: Path):
        """Receive the trajectory from the trajectory generator."""
        self.trajectory = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info(f"Received trajectory with {len(self.trajectory)} points.")

    def pose_callback(self, msg: PoseStamped):
        """Update the robot's current pose."""
        orientation_q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def control_loop(self):
        """Main control loop to follow the trajectory and handle phase transitions."""
        if not self.trajectory:
            self.get_logger().info("No trajectory to follow.")
            return
        if not self.current_pose:
            self.get_logger().info("Waiting for current pose.")
            return

        # Update phase dynamically
        self.update_phase()

        # Select target point based on the phase
        if self.current_phase == 1:
            target_point = self.parking_start
            linear_velocity = -0.2  # Reverse

        elif self.current_phase == 2:
            target_point = self.parking_middle
            linear_velocity = -0.1  # Slower reverse

        elif self.current_phase == 3:
            target_point = self.parking_end
            linear_velocity = 0.2  # Drive forward

        else:
            self.get_logger().warn("Invalid phase. Stopping.")
            self.publish_stop_command()
            return

        # Calculate control commands
        control_command = self.calculate_pid_control(target_point, linear_velocity)

        # Publish the control commands
        self.cmd_vel_publisher.publish(control_command)

    def update_phase(self):
        """Update the parking phase based on the robot's position."""
        if not self.current_pose:
            return

        current_x, current_y, _ = self.current_pose

        if self.current_phase == 1 and current_y <= self.parking_start[1]:
            self.current_phase = 2
            self.get_logger().info("Transitioned to Phase 2: Aligning with parking middle.")

        elif self.current_phase == 2 and current_y <= self.parking_middle[1]:
            self.current_phase = 3
            self.get_logger().info("Transitioned to Phase 3: Driving forward to center.")

    def calculate_pid_control(self, target_point, linear_velocity):
        """Calculate linear and angular velocity using a PID controller."""
        current_x, current_y, current_yaw = self.current_pose
        target_x, target_y = target_point

        # Calculate the heading error
        desired_yaw = math.atan2(target_y - current_y, target_x - current_x)
        yaw_error = desired_yaw - current_yaw

        # Normalize yaw error to the range [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # PID calculations
        self.integral_error += yaw_error
        derivative_error = yaw_error - self.previous_error

        angular_velocity = (
            self.kp * yaw_error +
            self.ki * self.integral_error +
            self.kd * derivative_error
        )

        # Update previous error
        self.previous_error = yaw_error

        # Create and return the control command
        control_command = Twist()
        control_command.linear.x = linear_velocity
        control_command.angular.z = angular_velocity

        return control_command

    def publish_stop_command(self):
        """Publish a stop command to halt the robot."""
        stop_command = Twist()
        self.cmd_vel_publisher.publish(stop_command)

def main(args=None):
    rclpy.init(args=args)
    node = ReverseParkingControllerWithPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
