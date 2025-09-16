import launch
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro
import re


def generate_launch_description():
    def remove_comments(text):
        # remove comments in urdf files
        pattern = r'<!--(.*?)-->'
        return re.sub(pattern, '', text, flags=re.DOTALL)
    # Define Paths
    pkg_share = FindPackageShare(package="tas2").find("tas2")
    pkg_scan_merger = FindPackageShare(package="ros2_laser_scan_merger").find("ros2_laser_scan_merger")
    model_path = os.path.join(pkg_share, "models/tas_car.urdf.xacro")
    model = remove_comments(xacro.process_file(model_path).toxml())
    rviz_path = os.path.join(pkg_share, "rviz/point_cloud.rviz")

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

    tas_hardware = Node(package="tas_hardware", executable="tas_hardware")

    return launch.LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
            scan_merger_launch
            # tas_hardware
        ]
    )
