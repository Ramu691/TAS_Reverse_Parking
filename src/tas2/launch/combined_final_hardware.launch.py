from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the path to the `tas2` package share directory
    pkg_share = FindPackageShare(package="tas2").find("tas2")

    # Paths to the individual launch files
    hardware_computer_launch = os.path.join(pkg_share, "launch", "hardware_computer.launch.py")
    hardware_navigation_launch = os.path.join(pkg_share, "launch", "hardware_navigation.launch.py")
    slam_launch = os.path.join(pkg_share, "launch", "slam.launch.py")
    custom_launch = os.path.join(pkg_share, "launch", "custom_Nodes.launch.py")

    # Include the launch files
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_computer_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_navigation_launch)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(custom_launch)
        ),
    ])
