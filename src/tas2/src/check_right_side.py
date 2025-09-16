#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from custom_msgs.msg import StateControl 
import numpy as np
from custom_msgs.msg import StateControl 

class FindPPSRightSide(Node):
    def __init__(self):
        super().__init__('FindPPSRightSide')

        # Subscription to /global_costmap/costmap
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,  # Message type
            '/global_costmap/costmap',  # Topic
            self.costmap_callback,  # Callback function
            10  # QoS depth
        )

        # Subscriber für StateControl
        self.state_control_sub = self.create_subscription(
            StateControl,
            '/state_control_topic',
            self.state_control_callback,
            10)


        # Publisher for /global_costmap/right_side
        self.right_side_publisher = self.create_publisher(
            OccupancyGrid,
            '/global_costmap/right_side',
            10
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # inits
        self.costmap_data = None
        self.costmap_info = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None

        # Timer recalculates and publishes costmap
        self.timer = self.create_timer(0.5, self.calculate_car_position) # every 0.5s

    def state_control_callback(self, msg):
        """Callback for StateControl messages"""
        self.wall_following_active = msg.wall_following
        self.rev_parking_active = msg.rev_parking
        # Close Node, once car is stopped and rev parking begins, since it is not needed anymore
        if not self.wall_following_active and (self.rev_parking_active==True):
            self.get_logger().info("Node is not needed anymore. Car is stopped and reverse parking initiated. Node is now closing...")
            self.control_timer.cancel()
            self.destroy_node()
            self.get_logger().info("Node destroyed")


    def costmap_callback(self, msg):
        #self.costmap_data = msg.data  # Costmap data (1D array)
        self.costmap_data = np.array(msg.data, dtype=np.int8)  # Convert to NumPy array
        self.costmap_data = self.costmap_data.reshape((msg.info.height, msg.info.width))

        self.costmap_info = msg.info  # Metadata: resolution, width, height, origin; check ros2 topic echo /global_costmap/costmap

        # origin of costmap is not static
        self.resolution = self.costmap_info.resolution
        self.origin_x = self.costmap_info.origin.position.x
        self.origin_y = self.costmap_info.origin.position.y

        # calculate car position in the map frame
        self.calculate_car_position()

    def state_control_callback(self, msg):
        """Callback for StateControl messages"""
        self.wall_following_active = msg.wall_following
        self.rev_parking_active = msg.rev_parking
        # Close Node, once car is stopped and rev parking begins, since it is not needed anymore
        if not self.wall_following_active and (self.rev_parking_active==True):
            self.get_logger().info("Node is not needed anymore. Car is stopped and reverse parking initiated. Node is now closing...")
            self.destroy_node()
            rclpy.shutdown()

    def calculate_car_position(self):
        # calculate car position in the map frame, transform to costmap coordinates
        try:
            # Get the car's transform in the map frame
            transform = self.tf_buffer.lookup_transform( # gets translation & rotation arguments (just like echo /tf)
                'map',  # Target frame
                'base',  # Source frame, check "ros2 run tf2_tools view_frames" for pdf overview
                rclpy.time.Time()  # Latest available transform, needed because use of buffer system
            )

            # Extract car position from transform
            car_x = transform.transform.translation.x
            car_y = transform.transform.translation.y

            # Calculate the car's grid position in the costmap
            if self.costmap_info:
            
                car_grid_x = int((car_x - self.origin_x) / self.resolution)
                car_grid_y = int((car_y - self.origin_y) / self.resolution)

                # filter relevant data
                cell_range_front = 2.0 / self.resolution # 1.5m to the front
                cell_range_back = 2.5 / self.resolution
                start_row = int(car_grid_x - cell_range_back)
                end_row = int(car_grid_x + cell_range_front)

                # check right side not exceeding costmap bounds
                if start_row < 0:
                    cell_range_back = cell_range_back + start_row
                    start_row = 0
                if end_row > self.costmap_data.shape[1]: # greater than y
                    end_row = self.costmap_data.shape[1]

                right_side_width = int(0.95 / self.resolution) # 1.5m to the right
                if right_side_width > car_grid_y:
                    right_side_width = car_grid_y
                #right_side_data = self.costmap_data[start_row:end_row, car_grid_x:] #old version, when KOSY not rotated
                right_side_data = self.costmap_data[car_grid_y-right_side_width:car_grid_y, start_row:end_row]

                # creates OccupancyGrid message type
                right_side_msg = OccupancyGrid()
                right_side_msg.header.frame_id = "map"
                right_side_msg.header.stamp = self.get_clock().now().to_msg()
                right_side_msg.info = self.costmap_info
                right_side_msg.info.height = right_side_data.shape[0]
                right_side_msg.info.width = right_side_data.shape[1]
                right_side_msg.info.origin.position.x = car_x - (cell_range_back) * self.resolution 
                #+ (car_grid_x) * self.resolution #self.origin_x + (car_grid_x) * self.resolution #old version, when KOSY not rotated
                right_side_msg.info.origin.position.y = car_y - (right_side_data.shape[0]+1) * self.resolution 
                #- int(cell_range_back * self.resolution) #self.origin_y + (car_grid_y-cell_range_back) * self.resolution #old version, when KOSY not rotated
                right_side_msg.data = right_side_data.flatten().tolist()

                # Publish the right-side costmap
                self.right_side_publisher.publish(right_side_msg)

                # uncomment for debugging info
                #self.get_logger().info("Published right-side costmap.")
                
            else:
                self.get_logger().warn("Costmap info not received yet. Cannot compute grid position.")

        except Exception as e:
            self.get_logger().warn(f"Failed to get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FindPPSRightSide()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
