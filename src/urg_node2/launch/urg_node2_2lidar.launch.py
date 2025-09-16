# Copyright 2022 eSOL Co.,Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# Launching multiple instances of urg_node2 when two LiDARs are connected. (front and back)

def generate_launch_description():

    # Loading the parameter file (front LiDAR)
    config_file_path_front = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_serial_front.yaml'
    )
    # Setting the path for the parameter file (back LiDAR)
    config_file_path_back = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_serial_back.yaml'
    )
    
    # Loading the parameter file (front LiDAR)    
    with open(config_file_path_front, 'r') as file:
        config_params_front = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # Loading the parameter file (back LiDAR)
    with open(config_file_path_back, 'r') as file:
        config_params_back = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # Starting urg_node2 as a lifecycle node (front LiDAR)
    lifecycle_node_front = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name_front'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name_front'))],
        parameters=[config_params_front],
        namespace='',
        output='screen',
    )

    # Starting urg_node2 as a lifecycle node (back LiDAR)
    lifecycle_node_back = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name_back'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name_back'))],
        parameters=[config_params_back],
        namespace='',
        output='screen',
    )

    # Transition from Unconfigure to Inactive state (performed if auto_start is true) (front LiDAR)
    urg_node2_node_front_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node_front,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_front),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Transition from Inactive to Active state (performed if auto_start is true) (front LiDAR)
    urg_node2_node_front_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node_front,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_front),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Transition from Unconfigure to Inactive state (performed if auto_start is true) (back LiDAR)
    urg_node2_node_back_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node_back,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_back),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Transition from Inactive to Active state (performed if auto_start is true) (back LiDAR)
    urg_node2_node_back_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node_back,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_back),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Parameters:
    # auto_start          : Automatically transition to Active state on startup (default)true
    # node_name_front       : Name of the front node (default) "urg_node2_front"
    # node_name_back       : Name of the back node (default)"urg_node2_back"
    # scan_topic_name_front : Topic name (default) "scan_front" *Not compatible with multi-echo*
    # scan_topic_name_back : Topic name (default) "scan_back" *Not compatible with multi-echo*
    # *If using multi-echo, directly edit the remappings in the node startup section above*
    # *Edit the parameter file name directly in the upper section for loaded parameter files*
   
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name_front', default_value='urg_node2_front'),
        DeclareLaunchArgument('node_name_back', default_value='urg_node2_back'),
        DeclareLaunchArgument('scan_topic_name_front', default_value='scan_front'),
        DeclareLaunchArgument('scan_topic_name_back', default_value='scan_back'),
        lifecycle_node_front,
        lifecycle_node_back,
        urg_node2_node_front_configure_event_handler,
        urg_node2_node_front_activate_event_handler,
        urg_node2_node_back_configure_event_handler,
        urg_node2_node_back_activate_event_handler,
    ])
