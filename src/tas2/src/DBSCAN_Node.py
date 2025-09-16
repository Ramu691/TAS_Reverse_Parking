#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
from geometry_msgs.msg import Point, Quaternion, PoseArray, Pose
from custom_msgs.msg import StateControl
from tf_transformations import quaternion_from_euler
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float32
 

class ParkingGapDetector(Node):
    def __init__(self):
        super().__init__('parking_gap_detector')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/right_side',
            self.costmap_callback,
            10
        )

        # Subscriber für StateControl
        self.state_control_sub = self.create_subscription(
            StateControl,
            '/state_control_topic',
            self.state_control_callback,
            10)

        self.marker_publisher = self.create_publisher(MarkerArray, '/parking_gaps', 10) # for visualization in Rviz
        self.gap_corners_publisher = self.create_publisher(PoseArray, '/parking_spot_corners', 10) # for parking Node
        self.gap_length_publisher = self.create_publisher(Float32, '/parking_gap_length', 10) # fpr parking Node
        self.boundary_boxes_publisher = self.create_publisher(PoseArray, '/boundary_boxes', 10) # for wall follwíng Node

        # Parameters for DBSCAN
        self.eps = 0.5
        self.min_samples = 10

        # Parameters of parking gap
        self.fixed_gap_width = 0.6
        self.min_gap_length = 1.3
        self.ideal_gap_length = 1.5
        self.gap_long_enough = False

    def state_control_callback(self, msg):
        """Callback for StateControl messages"""
        self.wall_following_active = msg.wall_following
        self.rev_parking_active = msg.rev_parking
        # Close Node, once car is stopped and rev parking begins, since it is not needed anymore
        if not self.wall_following_active and (self.rev_parking_active==True):
            self.get_logger().info("Node is not needed anymore. Car is stopped and reverse parking initiated. Node is now closing...")
            self.destroy_node()
            rclpy.shutdown()


    def get_aligned_bounding_box(self, points, rotation_matrix):
        """
        Computes the minimal oriented bounding box for a set of points.
        Returns the corners in world coordinates.
        """
        # Transform points into the rotated spac
        points_rotated = points @ rotation_matrix
        
        # Finde min/max in rotated coordinates
        min_coords = np.min(points_rotated, axis=0)
        max_coords = np.max(points_rotated, axis=0)
        
        # Create bounding box in rotated space
        bbox_rotated = np.array([
            [min_coords[0], min_coords[1]],  # bottom left
            [max_coords[0], min_coords[1]],  # bottom right
            [max_coords[0], max_coords[1]],  # top right
            [min_coords[0], max_coords[1]]   # top left
        ])
        
        # Transform back into world coordinates
        bbox_world = bbox_rotated @ rotation_matrix.T
        
        return bbox_world, bbox_rotated

    def state_control_callback(self, msg):
        """Callback for StateControl messages"""
        self.wall_following_active = msg.wall_following
        self.rev_parking_active = msg.rev_parking
        # Close Node, once car is stopped and rev parking begins, since it is not needed anymore
        if not self.wall_following_active and (self.rev_parking_active==True):
            self.get_logger().info("Node is not needed anymore. Car is stopped and reverse parking initiated. Node is now closing...")
            self.destroy_node()
            self.get_logger().info("Node destroyed")

    def publish_all_box_corners(self, obstacle_bboxes, parking_spot_corners):
        """Publishes all bounding boxes (obstacles and parking gaps)."""
        boundary_poses = PoseArray()
        boundary_poses.header.frame_id = "map"
        boundary_poses.header.stamp = self.get_clock().now().to_msg()

        #  Add the corners of the obstacle bounding boxes
        for bbox in obstacle_bboxes:
            for corner in bbox:
                pose = Pose()
                pose.position.x = float(corner[0])
                pose.position.y = float(corner[1])
                pose.position.z = 0.0
                boundary_poses.poses.append(pose)
        
        # Add the corners of the parking gaps
        for i in range(0, len(parking_spot_corners.poses), 4):  # Parklücken haben je 4 Eckpunkte
            for j in range(4):
                if i + j < len(parking_spot_corners.poses):
                    boundary_poses.poses.append(parking_spot_corners.poses[i + j])

        # Publish PoseArray on a separate topic
        self.boundary_boxes_publisher.publish(boundary_poses) # for wall following Node
        
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
    
    def create_gap_marker(self, start_point_world, length, fixed_gap_width, angle, marker_id):
        """
        Create filled and outline markers for parking gap.
        Parameters are in world coordinates, angle is the rotation of the principal axis.
        """
        if length < self.min_gap_length:
            color=(1.0,0.0,0.0,0.5)
            # return [], None  # No corners for gaps that are too small
            gap_long_enough = False

        elif self.min_gap_length <= length and length < self.ideal_gap_length:
            color=(1.0,1.0,0.0,0.5)
            gap_long_enough = True

        elif length >= self.ideal_gap_length:
            color=(0.0,1.0,0.0,0.5)
            gap_long_enough = True


        # Calculate corner points in the rotated space first
        corners_local = np.array([
            [0, 0],           # bottom left  (0) die Beschriftung stimmt nicht so ganz aber es funktioniert jetzt
            [0, -fixed_gap_width],      # top left  (1)
            [length, -fixed_gap_width], # top right  (2)
            [length, 0]       # bottom right (3)
        ])
        
        # Create rotation matrix
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
        
        # Transform corners to world coordinates
        corners_world = (corners_local @ rotation_matrix.T) + start_point_world
        
        # Create outline marker
        outline = self.create_marker(corners_world, Marker.LINE_STRIP, "gap_bounding_boxes", 
                                marker_id, color, scale=0.03)
        
        # Create filled marker
        filled = Marker()
        filled.header.frame_id = "map"
        filled.ns = "gap_transparent_rectangles"
        filled.id = marker_id + 1
        filled.type = Marker.CUBE
        filled.action = Marker.ADD
        
        # Calculate center point
        center_world = start_point_world + (rotation_matrix @ np.array([length/2, -fixed_gap_width/2]))
        
        filled.pose.position = Point(x=float(center_world[0]), y=float(center_world[1]), z=0.0)
        
        # Set orientation using the principal axis angle
        q = quaternion_from_euler(0, 0, angle)
        filled.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), 
                                        z=float(q[2]), w=float(q[3]))
        
        filled.scale.x = float(length)
        filled.scale.y = float(fixed_gap_width)
        filled.scale.z = 0.01
        
        filled_color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        filled.color = filled_color
        
        return [outline, filled], corners_world, gap_long_enough

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
        obstacle_corners_world = []
        
        for label in unique_labels:
            cluster = cluster_points[label]
            
            # Calculate bounding boxes
            bbox_world, bbox_rotated = self.get_aligned_bounding_box(
                cluster - center,  # centre points
                rotation_matrix
            )
            bbox_world += center  # transform back
            
            # save Bounding Boxes
            bboxes_rotated.append(bbox_rotated)
            obstacle_corners_world.append(bbox_world)
            
            # visualize Bounding Box
            markers.markers.append(self.create_marker(
                bbox_world, 
                Marker.LINE_STRIP,
                "obstacle_bounding_boxes",
                marker_id,
                color=(0.0,0.0,1.0,1.0)
            ))
            marker_id += 1

            self.get_logger().info(f"number of obstacle bounding boxes: {bboxes_rotated}")
        
        # Sort Bounding Boxes according to Position along the main axis
        boxes_with_pos = [(i, np.mean(box[:,0])) for i, box in enumerate(bboxes_rotated)]
        boxes_with_pos.sort(key=lambda x: x[1])

        # Initialize PoseArray for parking spot corners
        parking_gap_corners = PoseArray()
        parking_gap_corners.header.frame_id = "map"
        parking_gap_corners.header.stamp = self.get_clock().now().to_msg()

        # Initialize parking gap lenth
        parking_gap_length = Float32()
        
        # Check gaps between sorted boxes
        for i in range(len(boxes_with_pos)-1):
            current_idx = boxes_with_pos[i][0]
            next_idx = boxes_with_pos[i+1][0]
            
            # Calc size of gap in rotated space
            current_box = bboxes_rotated[current_idx]
            next_box = bboxes_rotated[next_idx]
            
            # Calc x coordinates of left and right edges
            '''current_left_x = np.min(current_box[:,0])'''
            current_right_x = np.max(current_box[:,0])
            next_left_x = np.min(next_box[:,0])

            # Find smallest y-coordinates within the gap
            # meaning between current_right_x and next_left_x
            current_right_points = current_box[current_box[:,0] >= current_right_x - 0.1]  # small tolerance
            next_left_points = next_box[next_box[:,0] <= next_left_x + 0.1]  # small tolerance

            current_bottom = np.max(current_right_points[:,1])
            next_bottom = np.max(next_left_points[:,1])

            # Use smaller one
            bottom_y = min(current_bottom, next_bottom)

            gap_start_rotated = np.array([current_right_x, bottom_y])
            gap_length = next_left_x - current_right_x  # Distance (length) to left corner of next Box
            if gap_length > 0:
                gap_start_world = (gap_start_rotated @ rotation_matrix.T) + center
                
                # Create gap marker
                gap_markers, corners, gap_long_enough = self.create_gap_marker(
                    gap_start_world,
                    gap_length,
                    self.fixed_gap_width, #parking_height,
                    angle,
                    marker_id
                )

                if gap_long_enough is False:
                    self.get_logger().info(f"no parking gap found")
                else:
                    self.get_logger().info(f"suitable parking gap found")
                
                if corners is not None and gap_long_enough is True:
                    # Add corner points to PoseArray 
                    for corner in corners:
                        pose = Pose()
                        pose.position.x = float(corner[0])
                        pose.position.y = float(corner[1])
                        pose.position.z = 0.0
                        # orientation along main axis
                        q = quaternion_from_euler(0, 0, angle)
                        pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]),
                                                   z=float(q[2]), w=float(q[3]))
                        parking_gap_corners.poses.append(pose)

                        parking_gap_length.data = gap_length
                
                    # Create additional markers for corners
                    corner_marker = Marker()
                    corner_marker.header.frame_id = "map"
                    corner_marker.ns = "parking_corners"
                    corner_marker.id = marker_id + 100  # Offset to avoid conflicts
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

                if gap_markers:  # Only if markers were created
                    markers.markers.extend(gap_markers)
                    marker_id += 2
                
                # Add distance label
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.ns = "gap_distance_labels"
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
        
        self.get_logger().info("/n")
        
        # Publishe all Markers und Coordinates
        self.gap_corners_publisher.publish(parking_gap_corners) # publish gap corners for parking
        self.gap_length_publisher.publish(parking_gap_length) # publish lenth of parking gap for parking
        self.publish_all_box_corners(obstacle_corners_world, parking_gap_corners) # publish all gap and obstacle corners for wall following
        self.marker_publisher.publish(markers) # publish markers for visualization

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

# TO DOS:
# 1. Publishen der 4 Koordinaten des grünen Rechtecks, wenn eine ausreichend große Lücke gefunden wurde - DONE
# 2. Publishen der Außenlinien der bounding boxes und Lücken für Line Following - DONE
# 3. ggf. Check, ob grünes Rechteck (Das evtl tiefer ist als bounding boxes wegen fester Höhe) tatsächlich komplett leer ist in der costmap