#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import ParkingStarting  
def create_reverse_parking_msg():
    """
    Create and populate the reverse parking message with predefined values.
    """
    msg = ParkingStarting()
    msg.rev_parking = True
    msg.start_x = 0.0
    msg.start_y = 0.0
    msg.start_heading = 0.0  # In radians
    msg.backward_distance = 0.0
    msg.radius_left = 0.8
    msg.radius_right = 0.8
    msg.forward_distance = 0.0
    msg.arc_angle_left = 65.0  # In degrees
    msg.arc_angle_right = 65.0  # In degrees
    msg.step_size = 0.05
    return msg

class ReverseParkingPublisher(Node):
    def __init__(self):
        super().__init__('reverse_parking_publisher')

        # Publisher for the ParkingStarting message
        self.publisher_ = self.create_publisher(ParkingStarting, '/parking_starting', 10)

        # Timer to ensure the message is published once
        self.timer = self.create_timer(1.5, self.publish_message)  # Publish after 1 second
        self.message_published = False

    def publish_message(self):
        if not self.message_published:
            msg = create_reverse_parking_msg()
            self.publisher_.publish(msg)
            self.get_logger().info("Published reverse parking message:")
            self.get_logger().info(f"Start (x, y, heading): ({msg.start_x}, {msg.start_y}, {msg.start_heading})")
            self.get_logger().info(f"Backward distance: {msg.backward_distance}, Forward distance: {msg.forward_distance}")
            self.get_logger().info(f"Radius left: {msg.radius_left}, Radius right: {msg.radius_right}")
            self.get_logger().info(f"Arc angles (left, right): ({msg.arc_angle_left}, {msg.arc_angle_right})")
            self.get_logger().info(f"Step size: {msg.step_size}")

            # Mark the message as published to avoid multiple publications
            self.message_published = True

            # Optionally shut down the node after publishing
            self.timer.cancel()
            self.get_logger().info("Message published. Shutting down node.")

def main(args=None):
    rclpy.init(args=args)
    node = ReverseParkingPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
