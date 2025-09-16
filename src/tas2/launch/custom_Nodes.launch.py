from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file to avoid starting all of the Nodes indivudally
Includes Nodes relevant for driving along parked cars, 
finding parking gaps, stopping correspondinly as well as driving backwards 
into the parking gap.
"""

def generate_launch_description():
    return LaunchDescription([
        # Node 1: check_right_side.py
        Node(
            package='tas2', 
            executable='check_right_side.py', 
            name='check_right_side', 
            output='screen' 
        ),

        # Node 2: DBSCAN_Node.py
        Node(
            package='tas2', 
            executable='DBSCAN_Node.py',  
            name='DBSCAN_Node', 
            output='screen'
        ),

        # Node 3: stop_drivig.py
        Node(
            package='tas2',  
            executable='stop_driving.py',  
            name='stop_drivig',  
            output='screen' 
        ),

        # Node 4: wall_following.py
        Node(
            package='tas2',  
            executable='wall_following.py', 
            name='wall_following',  
            output='screen'  
        ),

        # Node 5: goal_pose_publisher.py
        Node(
            package='tas2',  
            executable='goal_pose_publisher.py', 
            name='goal_pose_publisher',  
            output='screen'  
        )
    ])
