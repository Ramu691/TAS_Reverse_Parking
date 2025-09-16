import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Define Paths
    pkg_share = FindPackageShare(package='tas2').find('tas2')
    slam_config_file_path = os.path.join(pkg_share, 'config/slam.yaml')

    # Define and declare Launch parameter 
    use_simulation = LaunchConfiguration('simulation')
    declare_simulation = DeclareLaunchArgument(
        name="simulation",
        default_value='False',
        description="Either simulate (True) the TAS-Car in Gazebo or run it on the real hardware (False)."
    )

    # Add Slam Here 
    start_slam__node = Node(
        parameters=[
           slam_config_file_path,
          {'use_sim_time': use_simulation}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    return launch.LaunchDescription([
        declare_simulation,
        # robot_localization_node,
        start_slam__node,
    ])
