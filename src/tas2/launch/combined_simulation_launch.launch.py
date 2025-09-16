"""
File to launch whole project in Simulation. All required Nodes are started.
If this file is launched, no additional files need to be started.
Careful: all logging and debug info is printed in same single terminal.
"""
import os
import launch
import xacro
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    def remove_comments(text):
        pattern = r'<!--(.*?)-->'
        return re.sub(pattern, '', text, flags=re.DOTALL)

    # Package paths
    pkg_share = FindPackageShare(package="tas2").find("tas2")
    pkg_nav2 = FindPackageShare(package="nav2_bringup").find("nav2_bringup")
    pkg_gazebo = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_scan_merger = FindPackageShare(package="ros2_laser_scan_merger").find("ros2_laser_scan_merger")
    
    # Config paths
    nav2_config_file_path = os.path.join(pkg_share, "config/kartographie.yaml")
    map_yaml_file = os.path.join(pkg_share, "maps/dummy_map.yaml")
    # map_yaml_file = os.path.join(pkg_share, "maps/LSR_N5_basement.yaml")
    nav_to_pose_bt_path = os.path.join(pkg_share, "config/my_nav_to_pose_bt.xml")
    nav_through_poses_bt_path = os.path.join(pkg_share, "config/my_nav_through_poses_bt.xml")
    rviz_path = os.path.join(pkg_share, 'rviz/simulation.rviz')
    gazebo_world_path = os.path.join(pkg_share, 'world/parallel_parking_obstacles.world')
    
    # Gazebo model path
    gazebo_path = '/usr/share/gazebo-11/models/' + ':' + os.path.join(pkg_share, 'world/gazeboModels') 
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_path

    # Process URDF
    model_path = os.path.join(pkg_share, 'models/tas_car.urdf.xacro')
    model = remove_comments(xacro.process_file(model_path).toxml())

    # Launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    # Nav2 parameter substitutions
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": nav_to_pose_bt_path,
        "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml": nav_through_poses_bt_path,
    }
    
    nav2_config_file = RewrittenYaml(
        source_file=nav2_config_file_path,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # Define nodes and launches
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, "launch", "bringup_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "params_file": nav2_config_file,
            "map": map_yaml_file,
            "bt_navigator.default_nav_to_pose_bt_xml": nav_to_pose_bt_path,
            "bt_navigator.default_nav_through_poses_bt_xml": nav_through_poses_bt_path,
        }.items(),
    )

    # Nodes we wrote adressing the different stages
    """
    If we add more files or change filenames, we need to adapt
    this in this section!
    """
    parking_nodes = [
        Node(package='tas2', executable='check_right_side.py', name='check_right_side', output='screen'),
        Node(package='tas2', executable='DBSCAN_Node.py', name='DBSCAN_Node', output='screen'),
        Node(package='tas2', executable='stop_driving.py', name='stop_driving', output='screen'),
        Node(package='tas2', executable='wall_following.py', name='wall_following', output='screen'),
        Node(package='tas2', executable='goal_pose_publisher.py', name='goal_pose_publisher', output='screen'),
        # Node(package='tas2', executable='state_machine.py', name='state_machine', output='screen'),
        # Node(package='tas2', executable='forward_parking_correction.py', name='forward_parking_correction', output='screen'),        
    ]

    simulation_nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': model, 'use_sim_time': use_sim_time}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'world': gazebo_world_path}.items()
        ),
        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'tas_car',
                      '-x', '-0.75', '-y', '0.5', '-z', '0.0', '-Y', '1.57'],
            output='screen'
        ),
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["forward_position_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["forward_velocity_controller"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_scan_merger, "launch", "merge_2_scan.launch.py")),
            launch_arguments={"simulation": "True"}.items(),
        ),
        Node(
            package='tas_hardware', 
            executable='tas_simulation',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]

    return LaunchDescription([
        declare_use_sim_time,
        navigation_launch,
        *parking_nodes,
        *simulation_nodes
    ])