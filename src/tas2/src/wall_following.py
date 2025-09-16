#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformListener, Buffer
import math
from geometry_msgs.msg import PoseArray
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from custom_msgs.msg import StateControl
import time
from robot_utils import RobotUtilities # fct with robot position
from std_msgs.msg import Float32


class ParkingFollower(Node):
    def __init__(self):
        super().__init__('parking_follower')
        
        # to get robot position
        self.utilities = RobotUtilities(self)

        # Parameter
        self.desired_distance = 0.45  # desired distance to roadside in Meters
        self.control_frequency = 10.0  # Hz
        
        # State flags
        self.wall_following_active = True   # if true, wall_following algorithm is active
        
        # tf Buffers and Listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PID Parameters
        self.kp = 0.5 # hardware 0.5       
        self.ki = 0.0
        self.kd = 0.0
        # in simulation worked: kp = 0.9, ki = 0.005, kd = 0.1
        # in simulation worked maybe better: kp = 0.6, ki = 0.007, kd = 0.9

        # PID state variables
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()

        
        # Subscribers
        self.bbox_sub = self.create_subscription(
            PoseArray,
            '/boundary_boxes',
            self.bbox_callback,
            10)
            
        # Subscriber for StateControl
        self.state_control_sub = self.create_subscription(
            StateControl,
            '/state_control_topic',
            self.state_control_callback,
            10)
        
        # Publisher for movement of car (velocity)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)


        # Publisher for distance to wall
        self.wall_distance_pub = self.create_publisher(
            Float32,
            'wall_distance',
            10)
            
        # Timer for Mainloop
        self.timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        
        # Storage for current Bounding Boxes und selected line
        self.current_boxes = []
        self.selected_line = None

    def state_control_callback(self, msg):
        """Callback for StateControl messages"""
        self.wall_following_active = msg.wall_following
        
        # Log state changes
        self.get_logger().info(
            f'State updated - Wall Following: {self.wall_following_active},'
        )
        
        # once wall_following_active is set to false, car should stop
        # flag is set in stop_driving.py
        # stop_vehicle funtion is called
        if not self.wall_following_active:
            print('stop info received.. car should stop now')
            self.stop_vehicle()


    def stop_vehicle(self):
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
        self.get_logger().info("Node is not needed anymore. Car is stopped and reverse parking initiated. Node is now closing...")
        self.timer.cancel()
        self.destroy_node()
        self.get_logger().info("Node destroyed")
        
    def bbox_callback(self, msg):
        """Callback for new Bounding Box messages"""
        self.current_boxes = []  # List for grouped Bounding Boxes
        current_box = []

        for pose in msg.poses:
            # Extract position of bounding-box corners
            x = pose.position.x
            y = pose.position.y
            current_box.append([x, y])

            # Bounding box must consist of 4 corners
            if len(current_box) == 4:
                self.current_boxes.append(current_box)
                current_box = []  # Inits new bounding box
        
        # Debugging-log
        self.get_logger().info(f"Processed {len(self.current_boxes)} bounding boxes: {self.current_boxes}")
   
    def find_nearest_box(self, robot_pos):
        """Find next bounding box"""
        if not self.current_boxes:
            return None
            
        min_dist = float('inf')
        nearest_box = None
        
        for box in self.current_boxes:
            # Calculate distance to centre of bounding box
            center_x = sum(p[0] for p in box) / len(box)
            center_y = sum(p[1] for p in box) / len(box)
            dist = math.sqrt(
                (robot_pos[0] - center_x)**2 +
                (robot_pos[1] - center_y)**2
            )
            if dist < min_dist:
                min_dist = dist
                nearest_box = box
                
        return nearest_box
        
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
        
    def find_nearest_side(self, robot_pos, longest_sides):
        nearest_side = None
        min_distance = float('inf')

        for side in longest_sides:
            # Calculate distance of robot to each side
            for i in range(len(side)):
                line_start = side[i]
                line_end = side[(i + 1) % len(side)]  # Connect points
                dist = self.point_to_line_distance(robot_pos, line_start, line_end)

                if dist < min_distance:
                    min_distance = dist
                    nearest_side = side

        self.get_logger().info(f"Nearest side: {nearest_side}, Distance: {min_distance}")
        return nearest_side

    def follow_line(self, robot_pos, line):
        """Generate control commands to follow the line"""
        line_start, line_end = line
        # calc current distance
        current_distance = self.point_to_line_distance(robot_pos, line_start, line_end)
        
        # publisch current distance
        msg = Float32()
        msg.data = current_distance
        self.wall_distance_pub.publish(msg)

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

        angular_z = (
            self.kp*distance_error+
            self.ki*self.integral_error+
            self.kd*derivative_error
        )

        
        cmd.linear.x = 0.3  # constant forward movement
        #cmd.angular.z = 0.5 * distance_error  # P-Controller
        cmd.angular.z = angular_z

        # send updated command
        self.cmd_vel_pub.publish(cmd)
        self.previous_error=distance_error
        self.last_time=current_time

    def control_loop(self):
        """Main control loop for this Node"""
        # Check if wall_following is still needed
        if not self.wall_following_active:
            return
            
        # get robot position, return if not available    
        robot_pos = self.utilities.get_robot_position()
        if not robot_pos:
            return
        else:
            # Step 1: Find next bounding box
            nearest_box = self.find_nearest_box(robot_pos)
            if not nearest_box:
                self.get_logger().info('No bounding boxes found')
                return
                
            # Step 2: Find relevant side of bounding box
            longest_sides = self.get_longest_sides(nearest_box)
            self.selected_line = self.find_nearest_side(robot_pos, longest_sides)
            
            # Step 3: Steer accordingly
            if self.selected_line:
                self.follow_line(robot_pos, self.selected_line)
    


def main(args=None):
    rclpy.init(args=args)
    node = ParkingFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
