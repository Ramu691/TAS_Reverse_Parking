#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class LiDARHoughTransform(Node):
    def __init__(self):
        super().__init__('lidar_hough_transform')
        
        # Subscriber für die LiDAR-Daten
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                depth=10  # Optional: Puffergröße für Nachrichten
            )
        )
        
        # Publisher für die Linien-Marker
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

    def lidar_callback(self, scan):
        # Konvertiere die LaserScan-Daten in ein 2D-Bild
        resolution = 0.05  # Auflösung in Metern pro Pixel
        image_size = 500  # Größe des Bildes in Pixeln
        lidar_image = self.laserscan_to_image(scan, resolution, image_size)

        # Debug: Speichere das erzeugte Bild
        cv2.imwrite('/tmp/lidar_image.png', lidar_image)

        # Finde Kanten mit Canny
        edges = cv2.Canny(lidar_image, 50, 150)

        # Debug: Speichere die Kantenkarte
        cv2.imwrite('/tmp/lidar_edges.png', edges)

        # Wende Hough-Transformation an
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=30, maxLineGap=10)

        # Visualisiere die gefundenen Linien
        if lines is not None:
            self.get_logger().info(f'Number of lines detected: {len(lines)}')
            for line_id, line in enumerate(lines):
                x1, y1, x2, y2 = line[0]

                # Transformiere Bildkoordinaten in Weltkoordinaten
                wx1 = (x1 - image_size // 2) * resolution
                wy1 = (y1 - image_size // 2) * resolution
                wx2 = (x2 - image_size // 2) * resolution
                wy2 = (y2 - image_size // 2) * resolution

                # Erstelle einen Marker für RViz
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "hough_lines"
                marker.id = line_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.1  # Linienbreite
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                # Punkte der Linie
                marker.points = [
                    Point(x=float(wx1), y=float(wy1), z=0.0),
                    Point(x=float(wx2), y=float(wy2), z=0.0)
                ]

                # Veröffentliche den Marker
                self.marker_publisher.publish(marker)
        else:
            self.get_logger().info('Keine Linien gefunden')

    def laserscan_to_image(self, scan, resolution, image_size):
        """
        Konvertiert LaserScan-Daten in ein binäres Bild.
        """
        # Erstelle ein leeres Bild
        image = np.zeros((image_size, image_size), dtype=np.uint8)

        # Ursprung in der Mitte des Bildes
        origin = (image_size // 2, image_size // 2)

        # Iteriere über die LaserScan-Daten
        for angle, distance in enumerate(scan.ranges):
            if scan.range_min < distance < scan.range_max:
                # Winkel in Radiant
                theta = scan.angle_min + angle * scan.angle_increment

                # Kartesische Koordinaten berechnen
                x = int(origin[0] + (distance * np.cos(theta)) / resolution)
                y = int(origin[1] + (distance * np.sin(theta)) / resolution)

                # Setze den Punkt im Bild
                if 0 <= x < image_size and 0 <= y < image_size:
                    image[y, x] = 255  # Weißer Punkt für Hindernis

        return image

def main(args=None):
    rclpy.init(args=args)
    node = LiDARHoughTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
