import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():
    # Define Paths
    pkg_share = FindPackageShare(package="tas2").find("tas2")
    pkg_nav2 = FindPackageShare(package="nav2_bringup").find("nav2_bringup")
    nav2_config_file_path = os.path.join(pkg_share, "config/kartographie.yaml")
    map_yaml_file = os.path.join(pkg_share, "maps/dummy_map.yaml")
    nav_to_pose_bt_path = os.path.join(pkg_share, "config/my_nav_to_pose_bt.xml") # Behavior Tree
    nav_through_poses_bt_path = os.path.join(
        pkg_share, "config/my_nav_through_poses_bt.xml"
    )

    # Define and declare Launch parameter
    use_sim_time = LaunchConfiguration("use_sim_time") # for use of Gazebo
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    # Create our own temporary YAML files that include substitutions
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

    # robot localization via EKF
    # for sensor fusing
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Start navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py") # main Nav2 launch file, starts AMCL, planners, controllers etc.
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "params_file": nav2_config_file,
            "map": map_yaml_file,
            "bt_navigator.default_nav_to_pose_bt_xml": nav_to_pose_bt_path,
            "bt_navigator.default_nav_through_poses_bt_xml": nav_through_poses_bt_path,
        }.items(),
    )

    return launch.LaunchDescription(
        [
            declare_use_sim_time,
            #robot_localization_node,
            navigation_launch,
        ]
    )
