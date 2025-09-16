#!/usr/bin/env python3
from custom_msgs.msg import StateControl
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from robot_utils import RobotUtilities
from rclpy.action.client import ClientGoalHandle

class StateMachine(Node):
    def __init__(self):
        super().__init__('StateMachine')

        # Publisher for StateControl
        self.state_pub = self.create_publisher(
            StateControl,
            'state_control_topic',
            10
        )
        
        # probably not needed; replaced with /final_pose 
        # QoS profile to store only the most recent message
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # Keep only the last message
        )        
        # Subscription for bt_navigator msg
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            qos_profile)
        # Subscribe to navigation result        
        self.result = None
        self.last_processed_timestamp = 0
        self.last_goal_success_time = None

        self.active_goal_handles = {} 

        # Create a subscription to /final_pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/final_pose',
            self.final_pose_callback,
            10  # QoS depth
        )
        self.final_pose_x = None
        self.final_pose_y = None
        self.final_pose_tolerance = 0.15 # in meters

        # to get robot position, needed for line following
        self.utilities = RobotUtilities(self)
        self.robot_pos = None
        self.billo_state_forward = False

        # Timer for Mainloop
        self.change_state = False
        self.timer = self.create_timer(1, self.control_loop)

    def final_pose_callback(self, msg):
        self.final_pose_x = msg.pose.position.x
        self.final_pose_y = msg.pose.position.y

    def log_callback(self, msg):
        self.result = msg.msg
        current_timestamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        #self.get_logger().info(f"{self.result}")
        if current_timestamp > self.last_processed_timestamp:
            if "Goal succeeded" in msg.msg:
                self.last_processed_timestamp = current_timestamp +1
                self.last_goal_success_time = current_timestamp
                self.get_logger().info("Goal succeeded!!!")
                self.change_state = True

    def set_state_control(self, state, state_value):
        """Set state control"""
        state_msg = StateControl()
        setattr(state_msg, state, state_value)
        
        # state_msg.state = state_value

        self.state_pub.publish(state_msg)
        self.get_logger().info(f'{state} set to {state_value}')

    # STATE MACHINE LOGIC
    # vllt Logik in den einzelnen Nodes implementieren, nur Funktionen hier definiert lassen
    # nodes schalten sich dann selber aus und die naechste an

    def control_loop(self):
        # prevent error message before final_pose is even published
        if self.final_pose_x is None:
            return
        try:
            self.robot_pos = self.utilities.get_robot_position()
            robot_pos_x,robot_pos_y = self.robot_pos
            final_pose_x_error = abs(robot_pos_x - self.final_pose_x)
            final_pose_y_error = abs(robot_pos_y - self.final_pose_y)
            within_tolerance = (
                final_pose_x_error < self.final_pose_tolerance and 
                final_pose_y_error < self.final_pose_tolerance
            )       

            # Check if goal was recently achieved
            current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            goal_recently_succeeded = (
                self.last_goal_success_time is not None and
                (current_time - self.last_goal_success_time) < 2.5 
            )
            self.get_logger().info(f"within tolerance: {within_tolerance}, recent goal suc: {goal_recently_succeeded}") 
            if within_tolerance and goal_recently_succeeded:
                self.get_logger().info("Rev Parking completed, Change to forward correction")          
                self.set_state_control("rev_parking", False)
                self.set_state_control("forward_parking_correction", True)
                self.destroy_node()
                self.get_logger().info("Node destroyed")

        except Exception as e:
            self.get_logger().error(f"State Machine: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
