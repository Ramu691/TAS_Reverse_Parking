#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
from geometry_msgs.msg import Point, Quaternion, PoseArray, Pose
from tf_transformations import quaternion_from_euler
from std_msgs.msg import ColorRGBA

class ParkingGapDetector(Node):
    def __init__(self):
        super().__init__('parking_gap_detector')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        self.marker_publisher = self.create_publisher(MarkerArray, '/parking_gaps', 10)
        self.parking_corners_publisher = self.create_publisher(PoseArray, '/parking_spot_corners', 10)
        self.boundary_boxes_publisher = self.create_publisher(PoseArray, '/boundary_boxes', 10)
        
        # Parameters
        self.eps = 0.5
        self.min_samples = 10
        self.parking_width = 0.77  # Fixed width for parking spots
    
    def get_aligned_bounding_box(self, points, rotation_matrix):
        """
        Berechnet die minimal oriented bounding box für einen Satz von Punkten.
        Gibt die Eckpunkte in Weltkoordinaten zurück.
        """
        # Transformiere Punkte in den rotierten Raum
        points_rotated = points @ rotation_matrix
        
        # Finde min/max in rotierten Koordinaten
        min_coords = np.min(points_rotated, axis=0)
        max_coords = np.max(points_rotated, axis=0)
        
        # Erstelle Bounding Box im rotierten Raum
        bbox_rotated = np.array([
            [min_coords[0], min_coords[1]],  # links unten
            [max_coords[0], min_coords[1]],  # rechts unten
            [max_coords[0], max_coords[1]],  # rechts oben
            [min_coords[0], max_coords[1]]   # links oben
        ])
        
        # Transformiere zurück in Weltkoordinaten
        bbox_world = bbox_rotated @ rotation_matrix.T
        
        return bbox_world, bbox_rotated

    def publish_boundary_boxes(self, obstacle_bboxes, parking_spot_corners):
        """Publiziert alle Bounding Boxes (Hindernisse und Parklücken)."""
        boundary_poses = PoseArray()
        boundary_poses.header.frame_id = "map"
        boundary_poses.header.stamp = self.get_clock().now().to_msg()

        # Füge die Eckpunkte der Hindernis-Bounding-Boxes hinzu
        for bbox in obstacle_bboxes:
            for corner in bbox:
                pose = Pose()
                pose.position.x = float(corner[0])
                pose.position.y = float(corner[1])
                pose.position.z = 0.0
                boundary_poses.poses.append(pose)
        
        # Füge die Eckpunkte der Parklücken hinzu
        for i in range(0, len(parking_spot_corners.poses), 4):  # Parklücken haben je 4 Eckpunkte
            for j in range(4):
                if i + j < len(parking_spot_corners.poses):
                    boundary_poses.poses.append(parking_spot_corners.poses[i + j])

        self.boundary_boxes_publisher.publish(boundary_poses)

        # Visualisierung der Bounding Boxes in Rviz
        markers = MarkerArray()
        
        # Marker für die Hindernis-Bounding-Boxes
        obstacle_marker = Marker()
        obstacle_marker.header.frame_id = "map"
        obstacle_marker.ns = "obstacle_boxes"
        obstacle_marker.id = 0
        obstacle_marker.type = Marker.LINE_LIST
        obstacle_marker.action = Marker.ADD
        obstacle_marker.scale.x = 0.05
        obstacle_marker.color.r = 0.0
        obstacle_marker.color.g = 0.0
        obstacle_marker.color.b = 1.0
        obstacle_marker.color.a = 1.0

        # Verbinde die Eckpunkte jeder Hindernis-Box
        for bbox in obstacle_bboxes:
            for i in range(4):
                # Verbinde mit nächstem Punkt
                obstacle_marker.points.append(Point(x=float(bbox[i][0]), y=float(bbox[i][1]), z=0.0))
                obstacle_marker.points.append(Point(x=float(bbox[(i+1)%4][0]), y=float(bbox[(i+1)%4][1]), z=0.0))

        markers.markers.append(obstacle_marker)

        # Marker für die Parklücken-Bounding-Boxes
        parking_marker = Marker()
        parking_marker.header.frame_id = "map"
        parking_marker.ns = "parking_boxes"
        parking_marker.id = 1
        parking_marker.type = Marker.LINE_LIST
        parking_marker.action = Marker.ADD
        parking_marker.scale.x = 0.05
        parking_marker.color.r = 0.0
        parking_marker.color.g = 1.0
        parking_marker.color.b = 0.0
        parking_marker.color.a = 1.0

        # Verbinde die Eckpunkte jeder Parklücken-Box
        for i in range(0, len(parking_spot_corners.poses), 4):
            if i + 3 < len(parking_spot_corners.poses):
                for j in range(4):
                    start = parking_spot_corners.poses[i + j]
                    end = parking_spot_corners.poses[i + (j+1)%4]
                    parking_marker.points.append(start.position)
                    parking_marker.points.append(end.position)

        markers.markers.append(parking_marker)
        self.marker_publisher.publish(markers)

    def create_marker(self, points, marker_type, ns, marker_id, color=(0.0,0.0,1.0,1.0), scale=0.05):
        """Helper function to create basic markers."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        
        if points is not None:
            for point in points:
                marker.points.append(Point(x=float(point[0]), y=float(point[1]), z=0.0))
            # Close the loop for LINE_STRIP
            if marker_type == Marker.LINE_STRIP and len(points) > 0:
                marker.points.append(Point(x=float(points[0][0]), y=float(points[0][1]), z=0.0))
                
        return marker
    
    def create_gap_marker(self, start_point_world, length, width, angle, marker_id):
        """
        Create filled and outline markers for parking gap.
        Parameters are in world coordinates, angle is the rotation of the principal axis.
        """
        width = 0.6
        min_length = 1.3
        ideal_length = 1.5

        if length < min_length:
            color=(1.0,0.0,0.0,0.5)
            return [], None  # Keine Ecken für zu kleine Lücken

        elif min_length <= length and length < ideal_length:
            color=(1.0,1.0,0.0,0.5)

        elif length >= ideal_length:
            color=(0.0,1.0,0.0,0.5)

        # Calculate corner points in the rotated space first
        corners_local = np.array([
            [0, 0],           # links unten  (0)
            [0, -width],      # links oben   (1)
            [length, -width], # rechts oben  (2)
            [length, 0]       # rechts unten (3)
        ])
        
        # Create rotation matrix
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
        
        # Transform corners to world coordinates
        corners_world = (corners_local @ rotation_matrix.T) + start_point_world
        
        # Create outline marker
        outline = self.create_marker(corners_world, Marker.LINE_STRIP, "gap_rectangles", 
                                marker_id, color, scale=0.03)
        
        # Create filled marker
        filled = Marker()
        filled.header.frame_id = "map"
        filled.ns = "gap_rectangles_filled"
        filled.id = marker_id + 1
        filled.type = Marker.CUBE
        filled.action = Marker.ADD
        
        # Calculate center point
        center_world = start_point_world + (rotation_matrix @ np.array([length/2, -width/2]))
        
        filled.pose.position = Point(x=float(center_world[0]), y=float(center_world[1]), z=0.0)
        
        # Set orientation using the principal axis angle
        q = quaternion_from_euler(0, 0, angle)
        filled.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), 
                                        z=float(q[2]), w=float(q[3]))
        
        filled.scale.x = float(length)
        filled.scale.y = float(width)
        filled.scale.z = 0.01
        
        filled_color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        filled.color = filled_color
        
        return [outline, filled], corners_world

    def costmap_callback(self, msg):
        # Extract obstacle points
        height, width = msg.info.height, msg.info.width
        resolution = msg.info.resolution
        origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        
        cost_data = np.array(msg.data).reshape((height, width))
        obstacle_points = np.array([[origin[0] + x * resolution, origin[1] + y * resolution]
                                  for y, x in np.ndindex(height, width)
                                  if cost_data[y, x] == 100])
        
        if len(obstacle_points) == 0:
            self.get_logger().warning("No obstacles detected in costmap.")
            return
            
        # Cluster points
        dbscan = DBSCAN(eps=self.eps, min_samples=self.min_samples)
        labels = dbscan.fit_predict(obstacle_points)
        
        if np.all(labels == -1):
            self.get_logger().warning("No valid clusters found.")
            return
            
        # Get principal axis
        valid_points = obstacle_points[labels != -1]
        pca = PCA(n_components=2)
        pca.fit(valid_points)
        principal_axis = pca.components_[0]
        angle = np.arctan2(principal_axis[1], principal_axis[0])
        
        # Create rotation matrix
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
        
        # Create markers
        markers = MarkerArray()
        marker_id = 0
        
        # Add principal axis visualization
        center = np.mean(valid_points, axis=0)
        axis_points = np.array([center - principal_axis * 5, center, center + principal_axis * 5])
        markers.markers.append(self.create_marker(axis_points, Marker.LINE_STRIP, 
                                                "principal_axis", marker_id, 
                                                color=(1.0,1.0,0.0,0.8), scale=0.05))
        marker_id += 1
        
        # Group points by cluster
        unique_labels = np.unique(labels[labels != -1])
        cluster_points = {label: obstacle_points[labels == label] for label in unique_labels}
        
        # Process clusters
        bboxes_rotated = []
        bboxes_world = []
        
        for label in unique_labels:
            cluster = cluster_points[label]
            
            # Berechne Bounding Box
            bbox_world, bbox_rotated = self.get_aligned_bounding_box(
                cluster - center,  # Zentriere Punkte
                rotation_matrix
            )
            bbox_world += center  # Transformiere zurück
            
            # Speichere Bounding Boxes
            bboxes_rotated.append(bbox_rotated)
            bboxes_world.append(bbox_world)
            
            # Visualisiere Bounding Box
            markers.markers.append(self.create_marker(
                bbox_world, 
                Marker.LINE_STRIP,
                "bounding_boxes",
                marker_id,
                color=(0.0,0.0,1.0,1.0)
            ))
            marker_id += 1
        
        # Sortiere Bounding Boxes nach Position entlang der Hauptachse
        boxes_with_pos = [(i, np.mean(box[:,0])) for i, box in enumerate(bboxes_rotated)]
        boxes_with_pos.sort(key=lambda x: x[1])

        # Initialize PoseArray for parking spot corners
        parking_corners = PoseArray()
        parking_corners.header.frame_id = "map"
        parking_corners.header.stamp = self.get_clock().now().to_msg()
        
        # Überprüfe Lücken zwischen sortierten Boxes
        for i in range(len(boxes_with_pos)-1):
            current_idx = boxes_with_pos[i][0]
            next_idx = boxes_with_pos[i+1][0]
            
            current_box = bboxes_rotated[current_idx]
            next_box = bboxes_rotated[next_idx]
            
            current_right_x = np.max(current_box[:,0])
            next_left_x = np.min(next_box[:,0])

            current_right_points = current_box[current_box[:,0] >= current_right_x - 0.1]
            next_left_points = next_box[next_box[:,0] <= next_left_x + 0.1]

            current_bottom = np.max(current_right_points[:,1])
            next_bottom = np.max(next_left_points[:,1])

            bottom_y = min(current_bottom, next_bottom)

            gap_start_rotated = np.array([current_right_x, bottom_y])
            gap_length = next_left_x - current_right_x


            if gap_length > 0:
                gap_start_world = (gap_start_rotated @ rotation_matrix.T) + center
                
                # Erstelle Gap Marker mit der berechneten Höhe
                gap_markers, corners = self.create_gap_marker(
                    gap_start_world,
                    gap_length,
                    self.parking_width,
                    angle,
                    marker_id
                )
                
                if corners is not None:  # Nur für ausreichend große Lücken
                    # Füge die Eckpunkte zum PoseArray hinzu
                    for corner in corners:
                        pose = Pose()
                        pose.position.x = float(corner[0])
                        pose.position.y = float(corner[1])
                        pose.position.z = 0.0
                        # Setze Orientierung in Richtung der Hauptachse
                        q = quaternion_from_euler(0, 0, angle)
                        pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]),
                                                   z=float(q[2]), w=float(q[3]))
                        parking_corners.poses.append(pose)
                
                    # Erstelle zusätzliche Marker für die Ecken
                    corner_marker = Marker()
                    corner_marker.header.frame_id = "map"
                    corner_marker.ns = "parking_corners"
                    corner_marker.id = marker_id + 100  # Offset um Konflikte zu vermeiden
                    corner_marker.type = Marker.SPHERE_LIST
                    corner_marker.action = Marker.ADD
                    corner_marker.scale.x = 0.1
                    corner_marker.scale.y = 0.1
                    corner_marker.scale.z = 0.1
                    corner_marker.color.r = 1.0
                    corner_marker.color.g = 0.0
                    corner_marker.color.b = 1.0
                    corner_marker.color.a = 1.0
                    
                    for corner in corners:
                        point = Point()
                        point.x = float(corner[0])
                        point.y = float(corner[1])
                        point.z = 0.0
                        corner_marker.points.append(point)
                    
                    markers.markers.append(corner_marker)

                if gap_markers:  # Nur wenn Marker erstellt wurden
                    markers.markers.extend(gap_markers)
                    marker_id += 2
                
                # Füge Distanz-Label hinzu
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.ns = "distance_labels"
                text_marker.id = marker_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.text = f"{gap_length:.2f}m"
                
                label_pos_world = gap_start_world + (gap_length/2 * principal_axis)
                text_marker.pose.position.x = float(label_pos_world[0])
                text_marker.pose.position.y = float(label_pos_world[1])
                text_marker.pose.position.z = 0.5
                
                text_marker.scale.z = 0.3
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                markers.markers.append(text_marker)
                marker_id += 1
        
        # Publishe alle Marker und Koordinaten
        self.marker_publisher.publish(markers)
        self.parking_corners_publisher.publish(parking_corners)
        self.publish_boundary_boxes(bboxes_world, parking_corners)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingGapDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()