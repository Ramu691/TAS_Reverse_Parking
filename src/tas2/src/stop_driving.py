#!/usr/bin/env python3
# imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped, Twist
from custom_msgs.msg import StateControl 
from tf2_ros import TransformListener, Buffer
import math
from tf2_ros import TransformException
import time
from robot_utils import RobotUtilities # fct with robot position

################################ class
# Node to determine when to stop, based on received bounding box representing parking spot
class ParkingDetector(Node):
    def __init__(self):
        super().__init__('detector_stop_position_init_parking')

        # stop condition, y-coordinate parking space
        self.stop_offset = -0.2

        # Parklückeninformationen
        self.parking_start = None
        self.parking_length = None

        self.robot_pos = None
        self.corner_0 = None
        self.corner_1 = None
        self.corner_2 = None
        self.corner_3 = None

        # Coordinate system parameters
        self.cos_theta = None
        self.sin_theta = None
        self.coord_system_initialized = False

        # to get robot position
        self.utilities = RobotUtilities(self)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber
        # topic parking_spot_corners sends 4 points
        self.parking_box_corners_sub= self.create_subscription(
            PoseArray,
            '/parking_spot_corners',
            self.parking_box_corners,
            10
        )

        # Publisher for StateControl
        self.state_pub = self.create_publisher(
            StateControl,
            'state_control_topic',
            10
        )

        self.control_timer = self.create_timer(0.5, self.control_loop)

    def parking_box_corners(self, msg):
        # do nothing if not 4 corners detected
        if len(msg.poses) != 4:
            return

        # assign corners to data
        self.corner_0 = msg.poses[0].position
        self.corner_1 = msg.poses[1].position
        self.corner_2 = msg.poses[2].position
        self.corner_3 = msg.poses[3].position

        # Calculate Width and height of parking space detected based on parking box corners
        side_a = math.sqrt((self.corner_1.x - self.corner_0.x)**2 + (self.corner_1.y - self.corner_0.y)**2)
        side_b = math.sqrt((self.corner_2.x - self.corner_1.x)**2 + (self.corner_2.y - self.corner_1.y)**2)

        # safe longest side as length of parking space
        self.parking_length = max(side_a, side_b)

        # uncomment info to see calculated parking length
        # self.get_logger().info(f"Parklänge berechnet: {self.parking_length:.2f} m")

    
    # fct to build coordinate system with origin in parking space
    def create_coordinate_system_parking_space(self):
        """Initialize the parking space coordinate system."""
        if not all([self.corner_0, self.corner_1, self.corner_2, self.corner_3]):
            self.get_logger().warn("Cannot create parking space coordinate system - missing corner data")
            self.coord_system_initialized = False
            return False

        # Calculate parking space orientation vector (from corner_0 to corner_1)
        dx = self.corner_1.x - self.corner_0.x
        dy = self.corner_1.y - self.corner_0.y
        
        # Calculate angle between parking space and map coordinate system
        theta = math.atan2(dy, dx)
        
        # Create rotation matrix
        self.cos_theta = math.cos(theta)
        self.sin_theta = math.sin(theta)
        self.coord_system_initialized = True
        self.get_logger().info('Successfully calculated new coordinate system.')
        return True
    
    # Function to transform a point from map to parking space coordinates
    def transform_to_parking_space(self, point_x, point_y):
        # Translate point relative to origin (corner_0)
        translated_x = point_x - self.corner_0.x
        translated_y = point_y - self.corner_0.y
        
        # Rotate point to align with parking space
        parking_x = translated_x * self.cos_theta + translated_y * self.sin_theta
        parking_y = -translated_x * self.sin_theta + translated_y * self.cos_theta
        
        return parking_x, parking_y

    def control_loop(self):
        if self.parking_length is None or self.corner_0 is None:
            self.get_logger().warn("Waiting for parking space data...")
            return

        # get robot position, return if not available    
        current_pos = self.utilities.get_robot_position()
        if not current_pos:
            return
        
        self.robot_pos = current_pos

        # Initialize coordinate system if needed
        if not self.coord_system_initialized:
            if not self.create_coordinate_system_parking_space():
                self.get_logger().error("Failed to initialize coordinate system")
                return

        # Transform robot coordinates to parking space coordinates
        try:
            current_x_in_parking_coord, current_y_in_parking_coord = self.transform_to_parking_space(
                self.robot_pos[0], 
                self.robot_pos[1]
            )
            
            self.get_logger().info(
                f"Robot Position in parking space coordinates: x={current_x_in_parking_coord:.2f}, y={current_y_in_parking_coord:.2f}"
            )

            if current_y_in_parking_coord >= (self.parking_length + self.stop_offset):  # Stop at end of parking space + offset defined above
                self.get_logger().info("End of parking space reached. Stopping vehicle...")
                self.set_state_control()
                self.get_logger().info("Node is not needed anymore. Car is stopped and reverse parking initiated. Node is now closing...")
                # Stop the control timer
                self.control_timer.cancel()
                self.destroy_node()
                self.get_logger().info("Node destroyed")
                
        except Exception as e:
            self.get_logger().error(f"Error in coordinate transformation: {str(e)}")

    # To Do in this section: clean up and get rid of smaller redundancies --> check how it performs on real vehicle

    def set_state_control(self):
        """Set state control"""
        # command to stop wall following
        state_msg = StateControl()
        state_msg.wall_following = False

        self.state_pub.publish(state_msg)
        self.get_logger().info("wall_following stopped. Prepare parking...")

        # command to set rev_parking to True, acts as initiator for that process
        state_msg.rev_parking = True
        self.state_pub.publish(state_msg)
        self.get_logger().info("rev_parking set to True")


def main(args=None):
    rclpy.init(args=args)
    node = ParkingDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

