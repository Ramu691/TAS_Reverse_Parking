#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

# Node to detect potential parking spaces and mark them
# detection based on cost-map
# desicion based on vehicle size + buffer hardcoded

#### ToDo:
# Verhalten, wenn eine Lücke ist, aber sie zu groß ist und somit keine Parklücke sondern zbsp Straße, muss noch rein

class ParkingSpaceDetector(Node):
    def __init__(self):
        super().__init__('parking_space_detector')

        # Subscribe to the /local_costmap/costmap topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10
        )

        # Publisher for visualization in RViz
        self.marker_publisher = self.create_publisher(
            Marker,
            '/parking_spots_marker',
            10
        )

        # Vehicle dimensions in meters
        self.vehicle_length = 0.7  # 70 cm
        self.vehicle_width = 0.5   # 50 cm

    def costmap_callback(self, msg):
        self.get_logger().info("Received costmap data")
        parking_spots = self.detect_parking_spots(msg)
        
        # amount of parking spots we see here, correpsonds to amount of overlappung squares, which could fit our car. Currently not the actual amount of different parking spaces!
        if parking_spots:
            self.get_logger().info(f"Detected {len(parking_spots)} parking spots. These can be overlapping!")
            self.publish_markers(parking_spots, msg.info)
        else:
            self.get_logger().info("No parking spots detected.")

    def detect_parking_spots(self, msg):
        # Extract metadata
        resolution = msg.info.resolution # size of each cell
        width = msg.info.width
        height = msg.info.height

        # Convert 1D data array to 2D numpy array to transform grid into suitable form
        # grid has dimension of prev. defined units
        grid = np.array(msg.data).reshape((height, width))

        # dimensions in grid cells
        vehicle_length_cells = int(self.vehicle_length / resolution)
        vehicle_width_cells = int(self.vehicle_width / resolution)
        width_cells = int(width / resolution)
        height_cells = int(height / resolution)

        parking_spots = []

        # Scan the grid for free space
        # aktuelles Vorgehen: durch das grid vertikal, dann horizontal Durchgehen
        # da wir das Auto in der Mitte der costmap haben, und rechtsverkehrt herrscht, könne Parklücken nur rechts sein
        # es reicht also, wenn  wir nur die rechte Hälfte der costmap ansehen
        x_start = width_cells//2 # cast to int
        for y in range(height_cells - vehicle_width_cells):
            for x in range(x_start, width_cells - vehicle_length_cells):
                # Extract a subgrid representing the potential parking space
                subgrid = grid[y:y + vehicle_width_cells, x:x + vehicle_length_cells]

                ####### hier eine von zwei Versionen auswählen
                # Check if all cells in the subgrid are free
                if np.all(subgrid == 0):
                    parking_spots.append([(x, y), (x + vehicle_length_cells, y + vehicle_width_cells)]) # obere rechte und untere linke Ecke merken

                # Check if all cells in the subgrid are free
                # if np.all(subgrid == 0):
                #     # Add all coordinates in the subgrid to the parking spots list
                #     subgrid_coordinates = [
                #         (sub_x, sub_y)
                #         for sub_y in range(y, y + vehicle_width_cells)
                #         for sub_x in range(x, x + vehicle_length_cells)
                #     ]
                #     parking_spots.append(subgrid_coordinates)

        return parking_spots

    def publish_markers(self, parking_spots, info):
        marker = Marker()
        marker.header.frame_id = info.origin.position
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "parking_spots"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = info.resolution
        marker.scale.y = info.resolution
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for spot in parking_spots:
            world_x = info.origin.position.x + spot[0] * info.resolution
            world_y = info.origin.position.y + spot[1] * info.resolution

            point = Marker()
            point.pose.position.x = world_x
            point.pose.position.y = world_y
            point.pose.position.z = 0.05  # Slightly above the ground
            point.pose.orientation.w = 1.0

            marker.points.append(point.pose.position)

        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpaceDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
