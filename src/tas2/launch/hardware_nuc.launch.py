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
    urg_pkg_share = FindPackageShare(package="urg_node2").find("urg_node2")
    model_path = os.path.join(pkg_share, "models/tas_car.urdf.xacro")
    model = remove_comments(xacro.process_file(model_path).toxml())
    rviz_path = os.path.join(pkg_share, "rviz/hardware.rviz")
    urg_file_path = os.path.join(urg_pkg_share, "launch")

    urg_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urg_file_path + "/urg_node2_2lidar.launch.py")
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": model,
                "use_sim_time": False,
                "publish_frequency": 100.0,
            }
        ],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": False}],
    )

    tas_hardware = Node(package="tas_hardware", executable="tas_hardware")

    return launch.LaunchDescription(
        [
            # robot_state_publisher_node,
            # rviz_node,
            urg_node,
            tas_hardware,
        ]
    )
