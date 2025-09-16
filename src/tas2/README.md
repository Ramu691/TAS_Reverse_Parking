# TAS Project 
In the TAS Course, our team decided to work on **reverse parallel parking**.


## file structure
To ensure a smooth experience with the TAS2 software stacl, it's recommended to set up your workspace with the following structure. Please check whether your workspace looks like this:
```
├─ tas2                      # Main package for TAS2 simulator and hardware operation
│  ├─ config                 # Configuration files for localization, slam, control, and navigation
│  ├─ launch                 # Launch files for simulation and hardware
│  ├─ maps                   # maps
│  ├─ models                 # URDF vehicle model
│  ├─ rviz                   # RVIZ files
│  ├─ world                  # Simulation environment model
│  ├─ src                    # ROS2 nodes
```
---

## /config:

### kartographie.yaml

This file is a modified copy of **navigation.yaml**. This file initializes and sets up: amcl, bt_navigator, planner_server, controller_server, local_costmap, global_costmap, map_server, smoother_server, behavior_server, robot_state_publisher, waypoint_follower, velocity_smoother. 

---

## /launch:

### combined_final_hardware.launch.py

This launch file integrates multiple ROS2 launch files:`hardware_computer.launch.py`,`slam.launch.py`,`hardware_navigation.launch.py`,`custom_Nodes.launch.py`

---

### combined_simulation_launch.launch.py

Single launch file to start all necessary *.launch* files and nodes needed for parallel parking in Simulation. This is the easiest way to start our Programm and see what has been achieved throughout the Project Phase. This .launch file combines *custom_Nodes.launch.py, **simulation.launch.py* and *parking.launch.py*.

---

### custom_nodes.launch.py

Launch file to avoid starting all of the Nodes indivudally.
Includes Nodes relevant for driving along parked cars, 
finding parking gaps, stopping correspondinly as well as driving backwards into the parking gap. Outputs are displayed on the screen for easy debugging.

---

### parking.launch.py

Inspired by **navigation.launch.py**. Configured for the parallel parking use case.

---

### hardware_navigation.launch.py

Using asynch SLAM instead of default SLAM settings. SLAM is configured externally.

---

## /maps:

### dummy_map.yaml

This is a completly empty map. In the use case of parallel parking the environment is unknown, which means no prebuild map can be available. However a map file needs to be passed for a succesfull launch of both the simulation and hardware.

---

## /src:

### DBSCAN_Node.py

#### Overview
The `ParkingGapDetector` is a node designed to determine the orientation of the road and identify suitable parking gaps from an occupancy grid map. It uses clustering and bounding box analysis to detect and visualize parking spaces, publishing relevant data for other nodes in the system, such as reverse parking and wall-following nodes.

#### Subscribed Topics
1. **`/global_costmap/right_side`** (`nav_msgs/OccupancyGrid`)
   - Receives a cut-out of the occupancy grid map.
2. **`/state_control_topic`** (`custom_msgs/StateControl`)
   - Monitors system state to determine when to shut down the node.

#### Published Topics
1. **`/parking_gaps`** (`visualization_msgs/MarkerArray`)
   - Visualizes parking gaps and bounding boxes in RViz.
2. **`/parking_spot_corners`** (`geometry_msgs/PoseArray`)
   - Publishes the corners of detected parking gaps.
3. **`/parking_gap_length`** (`std_msgs/Float32`)
   - Publishes the length of the detected parking gap.
4. **`/boundary_boxes`** (`geometry_msgs/PoseArray`)
   - Publishes obstacle and parking gap boundary corners for wall-following.

#### Parameters
- **DBSCAN Clustering**:
  - `eps`: 0.5 (Neighborhood radius for clustering)
  - `min_samples`: 10 (Minimum points required to form a cluster)
- **Parking Gap Dimensions**:
  - `fixed_gap_width`: 0.6 meters (Fixed width of parking gap)
  - `min_gap_length`: 1.3 meters (Minimum acceptable length)
  - `ideal_gap_length`: 1.5 meters (Ideal parking gap length)

#### Workflow
1. **Costmap Processing and Clustering**:
   - Extracts obstacle points from the occupancy grid map.
   - Uses DBSCAN clustering to group nearby obstacle points into clusters.

2. **Principal Component Analysis (PCA)**:
   - Applies PCA to the clustered points to compute the principal axis (main direction) of the obstacles.
   - Align bounding boxes with the road's orientation.
   - Determining the gaps between obstacles along the road's orientation.

3. **Bounding Box Representation**
    Computes bounding boxes for each cluster, aligned with the road's orientation.

3. **Gap Analysis**:
   - Identifies gaps between adjacent bounding boxes.
   - Classifies gaps based on length and assigns colors:
        - **Red**: Too short.
        - **Yellow**: Acceptable but not ideal.
        - **Green**: Ideal for parking.
   - Creates visualization markers for gaps and bounding boxes.

4. **Data Publishing**:
   - Publishes the corners and lengths of parking gaps.
   - Publishes bounding box data for wall-following.
   - Publishes visualization markers for Rviz.

5. **State Monitoring**:
   - Monitors `StateControl` messages.
   - Shuts down the node when reverse parking is initiated, as it is no longer required.

#### Visualization in RViz with Markers
- Principal axis for clusters (yellow).
- Bounding boxes of obstacles (blue).
- Parking gaps (red/yellow/green based on length).
- Gap distance labels.

#### Dependencies
- **ROS 2**
- **Python Libraries**:
  - `numpy`
  - `scikit-learn`
  - `tf-transformations`
  - ROS 2 message packages: `nav_msgs`, `visualization_msgs`, `geometry_msgs`, `std_msgs`, `custom_msgs`

#### Limitations and Future Work
1.  Validate that the detected parking gap is fully empty in the costmap.
2.  Since the main axis is used to determine the orientation of all obstacles, the road must not be curved. A more advanced way ob determining the orientation of the obstacles ("parked cars") could be implemented.
3.  At the moment the node relies on the cut-out of the costmap because of following reasons:
    - To ensure that only obstacles on the right side the street are detected as obstacles
    - To ensure that abstacles that are right of the "parked cars" are not detected as such
    A more elegant way could be implemented, that also allows for a dynamic detection of the depth of the parking gap.

---

### check_right_side.py

#### Overview
The `FindPPSRightSide` node processes an occupancy grid map to extract and publish data relevant to the right side of the vehicle. It enables obstacle detection and environmental monitoring for navigation and wall-following functionalities. The node dynamically adapts based on the vehicle's position in the map frame and integrates seamlessly with the broader system.

#### Subscribed Topics
1. **`/global_costmap/costmap`** (`nav_msgs/OccupancyGrid`)
   - Receives the occupancy grid map for the environment.
2. **`/state_control_topic`** (`custom_msgs/StateControl`)
   - Monitors system state to determine node termination conditions.

#### Published Topics
1. **`/global_costmap/right_side`** (`nav_msgs/OccupancyGrid`)
   - Publishes the processed right-side occupancy grid map.

#### Parameters
- **Right-Side Dimensions**:
  - Range to the front: 2.0 meters.
  - Range to the back: 2.5 meters.
  - Range to the right: 0.95 meters.

#### Workflow
1. Costmap Subscription
- Subscribes to the occupancy grid map and stores its metadata.
- Converts the costmap data into a NumPy array for processing.

2. TF Lookup
- Retrieves the vehicle's transform in the map frame.
- Calculates the vehicle's grid position in the costmap.

3. Right-Side Data Filtering
- Extracts a section of the costmap based on the vehicle's position and the predefined dimensions.
- Adjusts boundaries to ensure data remains within costmap limits.

4. Data Publishing
- Converts the filtered right-side data into an occupancy grid message.
- Publishes the processed data to the `/global_costmap/right_side` topic.

5. State Monitoring
- Listens for `StateControl` messages to track wall-following and reverse parking states.
- Shuts down automatically when wall-following is deactivated and reverse parking begins.

#### Dependencies
- **ROS 2**
- **Python Libraries**:
  - `numpy`
  - ROS 2 message packages: `nav_msgs`, `geometry_msgs`, `custom_msgs`

---

### forward_parking_correction.py

### Overview
The `ForwardParkingCorrection` node assists in adjusting the vehicle's position after forward parking. It leverages LIDAR data, PID control, and TF transformations to align the vehicle precisely within the parking space. This node dynamically processes real-time data to ensure the vehicle remains centered and correctly oriented.

#### Subscribed Topics
1. **`/state_control_topic`** (`custom_msgs/StateControl`)
   - Monitors system state and enables/disables correction.
2. **`/parking_spot_corners`** (`geometry_msgs/PoseArray`)
   - Receives the detected parking space boundaries.
3. **`/scan`** (`sensor_msgs/LaserScan`)
   - Processes LIDAR data to determine obstacle distances.

#### Published Topics
1. **`cmd_vel`** (`geometry_msgs/Twist`)
   - Sends movement commands to align the vehicle correctly.
2. **`/line_forward`** (`visualization_msgs/MarkerArray`)
   - Publishes line markers for parking alignment visualization in RViz.
3. **`/lidar_front`** (`sensor_msgs/PointCloud2`)
   - Publishes processed LIDAR data for debugging.

#### Parameters
- **Controller Settings**:
  - **Control Frequency**: 10 Hz
  - **Desired Distance**: 0.235 meters (half of the car width)
- **PID Tuning**:
  - **kp**: 0.9
  - **ki**: 0.005
  - **kd**: 0.1
- **LIDAR Filtering**:
  - **Angle Range**: -40° to 40°
  - **Stop Distance**: 0.2 meters

#### Workflow
1. State Monitoring
- Listens for `StateControl` messages to activate or deactivate parking correction.

2. Parking Space Processing
- Extracts and analyzes parking space boundaries from `PoseArray`.
- Identifies the longest sides of the parking space for alignment reference.

3. LIDAR-Based Positioning
- Captures LIDAR scan data and extracts relevant frontal measurements.
- Converts LIDAR cutout into a `PointCloud2` message for debugging.

4. PID-Based Correction
- Calculates the deviation from the ideal position.
- Uses PID control to generate appropriate velocity commands.

5. Visualization & Debugging
- Publishes RViz markers to visualize alignment lines.
- Outputs LIDAR point cloud data for debugging.

6. Stopping Condition
- Monitors LIDAR data to detect close obstacles.
- Stops vehicle movement when alignment is achieved.

#### Dependencies
- **ROS 2**
- **Python Libraries**:
  - `numpy`
  - `math`
  - `struct`
  - `time`
  - ROS 2 message packages: `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `custom_msgs`

#### Limitations and Future Work
1. Enhance robustness for dynamic environments.
2. Optimize PID control for smoother alignment.
3. Improve handling of irregular parking spaces.

---

### goal_pose_publisher.py

#### Overview
This is a ROS2 node designed to generate and publish a reverse parking trajectory. It takes parking parameters from subscribed topics, calculates a trajectory for the robot, and publishes individual poses for the robot to follow. This node integrates seamlessly with reverse parking algorithm.

#### Subscribed Topics
1. **`/wall_distance`** (`std_msgs/Float32`)  
   Receives the distance to the wall for calculating the parking trajectory radius.

2. **`/parking_gap_length`** (`std_msgs/Float32`)  
   Provides the length of the detected parking gap.

3. **`/state_control_topic`** (`custom_msgs/StateControl`)  
   Monitors system state and determines when to initiate reverse parking.

#### Published Topics
1. **`/goal_pose`** (`geometry_msgs/PoseStamped`)  
   Publishes individual trajectory points for the reverse parking process.

#### Parameters
  - `car_length`: 0.65 meters (Length of the car for trajectory calculations).
  - `car_width`: 0.45 meters (Width of the car for trajectory calculations).
  - `Parking_gap_length` : Dynamic in length and used for the arc Angle calculation.
  - `start_x` and `start_y`: Initial robot position.
  - `Orientation`: Initial Heading of robot
  - `step_size`: 0.05 meters (Step size for trajectory point generation).
  - `distance_to_back_wall_of_gap`: 0.3 meters (Buffer distance to the back wall of the parking gap).
  - `radius_left` and `radius_right`: Radius for left and right arc movements.

#### Workflow
1. **Initialization**:
   - Initializes publishers and subscribers for required topics.
   - Sets up the TF listener to retrieve the robot's position.

2. **Parking Trajectory Generation**:
   - Subscribes to `/wall_distance` and `/parking_gap_length` to receive real-time parameters.
   - Computes trajectory points based on parking dimensions and parameters.

3. **Trajectory Publishing**:
   - Publishes individual trajectory points as `PoseStamped` messages to the `/goal_pose` topic.
   - Uses a timer to publish points sequentially at regular intervals.

4. **State Monitoring**:
   - Subscribes to `/state_control_topic` to monitor system state.
   - Only starts reverse parking when enabled via the `StateControl` message.

5. **Shutdown Handling**:
   - Shuts down publishing when all trajectory points have been sent or if the system state requires it.

####  RViz Visualization
- **Pose Markers**:
  - Displays individual trajectory points as markers.
  - Helps visualize the robot's planned reverse parking path.

---

### robot_utils.py

#### Overview
The `RobotUtilities` class provides reusable functions for determining the robot's position. 

#### Parameters
- **Transform Frames**:
  - Source: `map`
  - Target: `base_footprint`

#### Workflow
1. **Transform Lookup**:
   - Queries the `tf2` buffer to get the transformation between `map` and `base_footprint`.
   - Extracts the robot's position (x, y, z) and calculates its yaw angle.

2. **Error Handling**:
   - Logs errors if the transformation cannot be executed (e.g., due to timeout).

3. **Output**:
   - Returns the robot's position (x, y). Ignores the z-coordinate for simplicity.

#### Limitations and Future Work
1. Provide an optional parameter to include the yaw angle in the output.

---

### state_machine.py

#### Overview
The `StateMachine` node serves as the central controller for managing the system's navigation and parking states. It monitors navigation success, updates state control messages, and orchestrates the transition between reverse parking and forward parking correction.

#### Subscribed Topics
1. **`/rosout`** (`rcl_interfaces/Log`)
   - Monitors navigation logs to detect successful goal completion.

#### Published Topics
1. **`state_control_topic`** (`custom_msgs/StateControl`)
   - Publishes control messages to manage navigation and parking states.

#### Parameters
- **Timer Interval**: 5 seconds (checks for state transitions periodically).
- **QoS Settings**:
  - **Reliability**: Best effort.
  - **History**: Keep last message.
  - **Depth**: 1 (stores only the latest log entry).

#### Workflow
1. Navigation Monitoring
- Subscribes to `/rosout` logs.
- Extracts timestamps and checks for "Goal succeeded" messages.
- Updates internal state when a goal is successfully reached.

2. State Control Updates
- When reverse parking is completed, the node:
  - Publishes `rev_parking = False`.
  - Publishes `forward_parking_correction = True`.
  - Resets the transition flag.

3. State Transition Execution
- A periodic control loop checks if a transition is required.
- If a goal is completed, it triggers the next state in the parking sequence.

#### Dependencies
- **ROS 2**
- **Python Libraries**:
  - ROS 2 message packages: `rclpy`, `nav2_msgs`, `action_msgs`, `rcl_interfaces`, `custom_msgs`

#### Limitations and Future Work
1. Improve robustness for handling incomplete navigation goals.
2. Expand state management for additional states.

---

### stop_driving.py

#### Overview
The `ParkingDetector` node monitors the robot's position relative to a detected parking space. It determines when the robot should stop based on the parking space's geometry and initializes the reverse parking process.

#### Subscribed Topics
- **`/parking_spot_corners`** (`geometry_msgs/PoseArray`):  
  Receives the four corner points of the detected parking space.

#### Published Topics
- **`state_control_topic`** (`custom_msgs/StateControl`):  
  Updates the state machine to stop wall-following and initiate reverse parking.

#### Parameters
- **Stop Offset**:  
  `-0.2` meters (Stop position adjustment relative to the parking space length).

#### Workflow
1. Parking Space Initialization
- Subscribes to `/parking_spot_corners` to receive the parking space's bounding box data.
- Calculates the parking space's length and orientation.

2. Coordinate System Construction
- Creates a local coordinate system aligned with the parking space using its corners.

3. Position Transformation
- Transforms the robot's position from the global `map` frame to the parking space's local coordinate frame.

4. Stop Condition
- Monitors the robot's position relative to the end of the parking space.
- Stops the robot and shuts down the node when the stop condition is met.

5. State Control
- Publishes messages to stop wall-following and activate reverse parking.

---

### wall_following.py

#### Overview
The node is designed to enable the TAS-car to follow our imaginary road while maintaining a desired distance from the obstacles on the right side. The node processes bounding box data representing the obstacles and parking gaps. It uses PID control for alignment and distance regulation.

#### Subscribed Topics
1. **`/boundary_boxes`** (`geometry_msgs/PoseArray`)  
   - Receives bounding box data representing roadside obstacles.
2. **`/state_control_topic`** (`custom_msgs/StateControl`)  
   - Monitors system state to determine whether wall-following is active.

#### Published Topics
1. **`cmd_vel`** (`geometry_msgs/Twist`)  
   - Publishes velocity commands for robot movement.
2. **`wall_distance`** (`std_msgs/Float32`)  
   - Publishes the current distance from the wall for monitoring.

#### Parameters
- **Wall-Following**:
  - `desired_distance`: 0.45 meters (Ideal distance to maintain from the wall)
  - `control_frequency`: 10 Hz (Frequency of control loop updates)
- **PID Control**:
  - `kp`: 0.5 (Proportional gain for angular velocity control)
  - `ki`: 0.0 (Integral gain for angular velocity control)
  - `kd`: 0.0 (Derivative gain for angular velocity control)

#### Workflow

1. **Bounding Box Processing**:  
   - Extracts bounding box data from `/boundary_boxes`.
   - Identifies the nearest bounding box to the robot's current position and the boxes nearest side.
   - Computes the distance to the selected side.

2. **PID-Controlled Movement**:  
   - Uses a PID controller to regulate angular velocity based on the distance error.
   - Maintains a constant forward velocity for smooth wall-following.

3. **Distance Feedback**:  
   - Publishes the current distance from the wall for monitoring and debugging.

4. **State Monitoring and Shutdown**:  
   - Listens to `StateControl` messages to determine whether wall-following is active.
   - Stops the robot and shuts down the node when wall-following is disabled.

#### Dependencies
- **ROS 2**
- **Python Libraries**:
  - `numpy`
  - `math`
  - ROS 2 message packages: `geometry_msgs`, `std_msgs`, `custom_msgs`

#### Limitations and Future Work
**Simplicity**: The simple PID controller could be replaced with more advanced path planning and controlling algorithms.
The cutout from the built occupancy grid (check_right_side.py) is fixed in size. If at the start Position of the Vehicle obstacles are behind, they influence the wall_following.py node in such a way, that the created axis through the bounding boxes, which is used as a reference for the PID-Controller, is manipulated. One potential improvement was started in the feature branch feature-ignore-branch, such that the cutout of the occupancy grid grows in size with the distance travelled. However, this could not be fully implemented due to time restrictions.

---

## /world:

### complex_parking.world

#### Overview
As the name already suggests, this is another simulation world, but more complex. This file contains even more obstacles and blocks representing parked cars. Furthermore, the cars are aligned such that they represent a street that is bent. Used to check edge cases and performance stress test. Due to the final functionality of **DBSCAN_Node.py**, an actual car cannot stably drive on a bent street.

---

### parallel_parking.world

#### Overview
Very simple simulation world. Used for the first phase of testing of parallel parking in rviz and Gazebo.<br>
Simulation needs to be in a **.world** file (.sdf has worse performance).


---

### parallel_parking_firstPPS_small.world

#### Overview
The `parallel_parking_firstPPS_small.world` file defines a Gazebo Simulation world with a 1m gap to test the passing functionality of small parking spaces. Used for testing of parallel parking in rviz and Gazebo.


---

### parallel_parking_obstacles.world

#### Overview
The `parallel_parking_obstacles.world` file defines a Gazebo Simulation world with more obstacles which allows for better SLAM performance. Used for testing of parallel parking in rviz and Gazebo.
