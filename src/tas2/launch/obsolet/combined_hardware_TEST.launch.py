import launch
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import os
import xacro
import re


def generate_launch_description():
    def remove_comments(text):
        # Remove comments in URDF files
        pattern = r'<!--(.*?)-->'
        return re.sub(pattern, '', text, flags=re.DOTALL)

    # Define Paths
    pkg_share = FindPackageShare(package="tas2").find("tas2")
    pkg_nav2 = FindPackageShare(package="nav2_bringup").find("nav2_bringup")
    pkg_scan_merger = FindPackageShare(package="ros2_laser_scan_merger").find("ros2_laser_scan_merger")
    # urg_file_path = os.path.join(urg_pkg_share, "launch")

    nav2_config_file_path = os.path.join(pkg_share, "config/navigation_hardware.yaml")
    map_yaml_file = os.path.join(pkg_share, "maps/LSR_N5_basement.yaml")
    #map_yaml_file = os.path.join(pkg_share, "maps/dummy_map.yaml")
   
    nav_to_pose_bt_path = os.path.join(pkg_share, "config/my_nav_to_pose_bt.xml")
    nav_through_poses_bt_path = os.path.join(pkg_share, "config/my_nav_through_poses_bt.xml")

    model_path = os.path.join(pkg_share, "models/tas_car.urdf.xacro")
    model = remove_comments(xacro.process_file(model_path).toxml())
    rviz_path = os.path.join(pkg_share, "rviz/point_cloud.rviz")

    # slam_params = os.path.join(
    #     FindPackageShare("slam_toolbox").find("slam_toolbox"),
    #     "config",
    #     "mapper_params_online_async.yaml"
    # )

    # Define and declare Launch parameter
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    declare_slam = DeclareLaunchArgument(
        "slam", default_value="True", description="Whether run a SLAM"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": nav_to_pose_bt_path,
        "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml": nav_through_poses_bt_path,
        "amcl.ros__parameters.initial_pose.x": "0.0", #-0.0127192
        "amcl.ros__parameters.initial_pose.y": "0.0", #-0.0600633
        "amcl.ros__parameters.initial_pose.z": "0.0",
        "amcl.ros__parameters.initial_pose.yaw": "0.0", #0.077211 #3.218803
    }

    nav2_config_file = RewrittenYaml(
        source_file=nav2_config_file_path,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # Start navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": slam,
            "use_sim_time": use_sim_time,
            "params_file": nav2_config_file,
            "map": map_yaml_file,
            "bt_navigator.default_nav_to_pose_bt_xml": nav_to_pose_bt_path,
            "bt_navigator.default_nav_through_poses_bt_xml": nav_through_poses_bt_path,
        }.items(),
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": False}],
    )

    scan_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scan_merger, "launch", "merge_2_scan.launch.py"),
        ),
        launch_arguments={"simulation": "False"}.items(),
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": model,
                "use_sim_time": False,
                # "publish_frequency": #100.0,
            }
        ],
    )

    parking_nodes = [
        Node(package='tas2', executable='check_right_side.py', name='check_right_side', output='screen'),
        Node(package='tas2', executable='DBSCAN_Node.py', name='DBSCAN_Node', output='screen'),
        Node(package='tas2', executable='stop_driving.py', name='stop_drivig', output='screen'),
        Node(package='tas2', executable='wall_following.py', name='wall_following', output='screen'),
        Node(package='tas2', executable='goal_pose_publisher.py', name='goal_pose_publisher', output='screen')
    ]

    return launch.LaunchDescription(
        [
            declare_use_sim_time,
            declare_slam,
            navigation_launch,
            rviz_node,
            scan_merger_launch,
            robot_state_publisher_node,
            *parking_nodes           
        ]
    )
