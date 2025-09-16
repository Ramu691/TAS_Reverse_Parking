#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseArray, TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import StateControl
import numpy as np
from robot_utils import RobotUtilities
import math
import struct
from std_msgs.msg import Float32
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from state_machine import StateMachine
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField

class ForwardParkingCorrection(Node):
    def __init__(self):
        super().__init__('ForwardParkingCorrection')
        
        # inits
        self.corner_0 = None
        self.corner_1 = None
        self.corner_2 = None
        self.corner_3 = None
        self.parking_space = []
        self.stop_distance = 0.38 # Lidar is not reliable underneath approx 40cm
        self.lidar_front = None
        self.forward_parking_correction = False

        # Controller Parameters
        self.desired_distance = 0.50/2  # desired distance half the car width
        self.control_frequency = 10.0  # Hz
        # PID Values
        self.kp = -4       
        self.ki = 0.0 #0.005
        self.kd = 0.0 #0.1
        # PID state variables
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()

        # tf Buffers and Listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # to get robot position, needed for line following
        self.utilities = RobotUtilities(self)

        # Publisher for StateControl
        self.state_pub = self.create_publisher(
            StateControl,
            'state_control_topic',
            10
        )
        # Subscriber for StateControl
        self.state_control_sub = self.create_subscription(
            StateControl,
            '/state_control_topic',
            self.state_control_callback,
            10)
        
        # Subscriber to topic parking_spot_corners, sends 4 points
        self.parking_box_corners_sub= self.create_subscription(
            PoseArray,
            '/parking_spot_corners',
            self.parking_space_callback,
            10
        )

        # QoS Profile, needed for /scan topic
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match QoS of the /scan topic
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber to the LaserScan topic
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        # Publisher for movement of car (velocity)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        self.line_publisher = self.create_publisher(MarkerArray, '/line_forward', 10) # for visualization in Rviz

        # Timer for Mainloop
        self.timer = self.create_timer(1.0/self.control_frequency, self.control_loop)       

        # for debugging, piblish lidar cutour
        self.lidar_publisher = self.create_publisher(PointCloud2, '/lidar_front', 10)

    def state_control_callback(self, msg):
        """Callback for StateControl messages"""
        self.forward_parking_correction = msg.forward_parking_correction
        # Log state changes
        self.get_logger().info(
            f'Forward parking correction set to: {self.forward_parking_correction}, '
        )

    def lidar_callback(self, msg):
        # LIDAR information
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        lidar_raw = np.array(msg.ranges)
        # Filter out invalid data (inf or NaN)
        lidar_valid = lidar_raw[np.isfinite(lidar_raw)]

        # get LIDAR cutout in front of the car
        start_angle = -50.0 # 0 degrees is straight ahead
        end_angle = 50.0
        start_index = int((math.radians(start_angle) - angle_min) / angle_increment)
        end_index = int((math.radians(end_angle) - angle_min) / angle_increment)
        self.lidar_front = lidar_valid[start_index:end_index]

        # for debugging
        # Convert to XYZ points
        points = []
        for i, r in enumerate(self.lidar_front ):
            angle = angle_min + (start_index + i) * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0  # 2D LiDAR usually has no Z component
            points.append(struct.pack("fff", x, y, z))

        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = msg.header
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12  # 3 floats * 4 bytes
        pc_msg.row_step = 12 * len(points)
        pc_msg.data = b"".join(points)
        pc_msg.is_dense = True
        self.lidar_publisher.publish(pc_msg)

    def parking_space_callback(self, msg):
        """Callback for new Bounding Box messages"""

        for pose in msg.poses:
            # Extract position of bounding-box corners
            x = pose.position.x
            y = pose.position.y
            self.parking_space.append([x, y])
        
        # Debugging-log
        #self.get_logger().info(f"parking space: {self.parking_space}")

    def get_longest_sides(self, box):
        """Extract longest side of box (assumes cars are parked parallel to street)"""
        sides = []
        for i in range(len(box)):
            p1 = box[i]
            p2 = box[(i+1) % len(box)]
            length = math.sqrt(
                (p2[0] - p1[0])**2 +
                (p2[1] - p1[1])**2
            )
            sides.append((length, (p1, p2)))
            
        # Sort length and take two longest
        sides.sort(key=lambda x: x[0], reverse=True)
        return [s[1] for s in sides[:2]]
        
    def get_left_side(self, longest_sides):
        left_side = []
        if longest_sides[0][0][0] < longest_sides[1][0][0]:
            left_side = longest_sides[0]
        else:
            left_side = longest_sides[1]

        #self.get_logger().info(f"left_side coordinates: {left_side}")
        return left_side

    def stop_vehicle(self): #copied from wall_following.py 
        """stop all movements of car right where it stands --> no correction in terms of positioning is happening here"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Vehicle stopped')
        self.destroy_node()
        self.get_logger().info("Node destroyed")

    def point_to_line_distance(self, point, line_start, line_end):
        # Distance point-line (2D)
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end

        # Linevector
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:  # line is a point
            return math.sqrt((px - x1)**2 + (py - y1)**2)

        # Projektion of the point onto the line
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx**2 + dy**2)))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy

        # Distance to that point 
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    def follow_line(self, robot_pos, line):
        """Generate control commands to follow the line"""
        line_start, line_end = line
        # calc current distance
        current_distance = self.point_to_line_distance(robot_pos, line_start, line_end)

        # create and send movement commands
        cmd = Twist()
        
        # Simple P-Controller for distance control
        #distance_error = self.desired_distance - current_distance
        
        # PID control
        current_time = time.time()
        delta_time = current_time-self.last_time

        # calc errors
        distance_error = self.desired_distance-current_distance
        self.integral_error += distance_error*delta_time
        derivative_error = (distance_error-self.previous_error)/delta_time

        self.get_logger().info(f"distance error: {distance_error}")

        angular_z = (
            self.kp*distance_error+
            self.ki*self.integral_error+
            self.kd*derivative_error
        )

        
        cmd.linear.x = 0.2  # constant forward movement
        #cmd.angular.z = 0.5 * distance_error  # P-Controller
        cmd.angular.z = angular_z

        # send updated command
        self.cmd_vel_pub.publish(cmd)
        self.previous_error=distance_error
        self.last_time=current_time

    def publish_line_marker_viz(self, coordinates):
        marker_array = MarkerArray()
        line_marker = Marker()

        # Setup marker properties
        line_marker.header.frame_id = "map"
        line_marker.ns = "following line"
        line_marker.id = 20
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # Line thickness
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        point1 = Point(x=coordinates[0][0], y=coordinates[0][1], z=0.0)
        point2 = Point(x=coordinates[1][0], y=coordinates[1][1], z=0.0)
        line_marker.points.append(point1)
        line_marker.points.append(point2)

        marker_array.markers.append(line_marker)
        self.line_publisher.publish(marker_array)
        #self.get_logger().info('Published line markers.')
        
    def set_state_control(self, state, state_value):
        """Set state control"""
        state_msg = StateControl()
        setattr(state_msg, state, state_value)
        
        # state_msg.state = state_value

        self.state_pub.publish(state_msg)
        self.get_logger().info(f'{state} set to {state_value}')

    def control_loop(self):
        """Main control loop for this Node"""
        try:
            if self.forward_parking_correction:
                print('Backward process completed. Driving forward and aligning now.')
                # if not self.forward_parking_correction:
                #     return
                
                # get robot position, return if not available    
                robot_pos = self.utilities.get_robot_position()
                if not robot_pos:
                    return

                longest_sides = self.get_longest_sides(self.parking_space)
                #self.get_logger().info(f"longest sides: {longest_sides}")
                left_side = self.get_left_side(longest_sides)
                self.publish_line_marker_viz(left_side)
                if left_side:
                    print('forward straiting now') 
                    self.get_logger().info(f"Obstacle detected within {min(self.lidar_front)} meters")                     
                    self.follow_line(robot_pos, left_side)
                    if min(self.lidar_front) < self.stop_distance:
                        self.get_logger().info(f"Obstacle detected within {self.stop_distance} meters. Robot stopping.")
                        self.set_state_control("forward_parking_correction", False)
                        self.stop_vehicle() # needs to be last, destroys node
                    
        except Exception as e:
            self.get_logger().error(f"Forward Parking Correction: {str(e)}")
    
def main(args=None):
    rclpy.init(args=args)
    node = ForwardParkingCorrection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()