#!/usr/bin/env python3
import math
import rclpy  
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import TransformStamped

"""
Module to avoid redundantly using same get_robot_position in different Nodes. 
"""

class RobotUtilities:
    def __init__(self, node: Node):
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def get_robot_position(self):
        try:
            # Transformation between map and base_link
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )
            # Extract position and orientation
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            # self.node.get_logger().info(
            #     f'Robot Position - X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, Yaw: {math.degrees(yaw):.2f}°'
            # )

            return x, y  # Only x and y are needed in these nodes
        except TransformException as ex:
            self.node.get_logger().error(
                f'Could not execute transformation: {str(ex)}'
            )
            return None
    
