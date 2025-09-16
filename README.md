# TAS Project Group A, Wintersemester 2024/2025
In the TAS Course of 2024/2025, our team decided to work on **reverse parallel parking**.

Our project proposal can be found in our project folder:
https://gitlab.lrz.de/tas/tas_ws2425/tas-projects/group-a/original_project/-/blob/main/Project%20Proposal.pdf?ref_type=heads

Here you can see videos of the final result of our work:
- Video of the real parking TAS-car: https://youtube.com/shorts/lSAyZmGRASM
- Corresponding Visualization in Rviz: https://youtu.be/g1yHyOvIkZ0

## Contributing Students:
1. **Florian Samuel Blacha**
2. **Ramkrishna Chaudhari**
3. **Johannes Grandien**
4. **Phil-Martin Kaulfuss**

## file structure

```
Tas Project                  
├─ hardware                     
├─ src                          
│  ├─ custom_msgs             # this packages was created to be able to use custom defined message types
│  ├─ ros2_laser_scan_merger    
│  ├─ tas_hardware               
│  ├─ tas2                      
│  │  ├─ config                
│  │  ├─ launch                 
│  │  ├─ maps                   
│  │  ├─ models                 
│  │  ├─ rviz
│  │  ├─ src                  # this folder contains all our custom build nodes                
│  │  ├─ world             
│  ├─ urg_node2                 
├─ README.md                   
```

---

## How to start the Demos

### Simulation

To run the demo, we created a combined launch file, that launches the simulation environment, navigation & mapping and all custom nodes:
```bash
ros2 launch tas2 combined_simulation_launch.launch.py
```

To make debugging easier, the simulation, navigation and custom nodes can be run in a separate launch files:
```bash
ros2 launch tas2 simulation.launch.py
```
```bash
ros2 launch tas2 parking.launch.py
```
```bash
ros2 launch tas2 custom_Nodes.launch.py
```

In case there occur errors when running the custom_Nodes.launch.py, our custom Nodes can be run separately. Instead of custom_Nodes.launch.py, run:
```bash
ros2 run tas2 check_right_side.py
```
```bash
ros2 run tas2 DBSCAN_Node.py
```
```bash
ros2 run tas2 check_right_side.py
```
```bash
ros2 run tas2 stop_driving.py
```
```bash
ros2 run tas2 wall_following.py
```
```bash
ros2 run tas2 goal_pose_publisher.py
```

---

### Hardware

1. Connect to the TAS Car's WiFi network using the following command:
```bash
ssh Group_A@IP_ADDRESS_OF_CAR_
```

2. Launch the hardware components on the TAS-Car computer by running:
```bash
ros2 launch tas2 hardware_nuc.launch.py
```

3. to run the demo, we created a combined launch file, that launches the simulation environment, navigation & mapping and all custom nodes:
```bash
ros2 launch tas2 combined_final_hardware.launch.py
```

To make debugging easier, the hardware computer, navigation and custom nodes can be run in separate launch files:
```bash
ros2 launch tas2 hardware_computer.launch.py
```
```bash
ros2 launch tas2 hardware_navigation.launch.py
```
```bash
ros2 launch tas2 slam.launch.py
```
```bash
ros2 launch tas2 custom_Nodes.launch.py
```

In case there occur errors when running the custom_Nodes.launch.py, our custom Nodes can be run separatly. Instead of custom_Nodes.launch.py, run:
```bash
ros2 run tas2 check_right_side.py
```
```bash
ros2 run tas2 DBSCAN_Node.py
```
```bash
ros2 run tas2 check_right_side.py
```
```bash
ros2 run tas2 stop_driving.py
```
```bash
ros2 run tas2 wall_following.py
```
```bash
ros2 run tas2 goal_pose_publisher.py
```

## Dependencies:
All requirements can be found in the requirements.txt file. 

## Restrictions
- It was decided in our corresponding project proposal that our use case starts with cars being parked on the right side of the TAS car and us searching for a parking space. When starting the program, one must thus ensure that on the right side of the car is some kind of fixed obstacle.
- Once a parking space is classified as such, it is not checked anymore whether the gap is still free.
- The car follows obstacles placed on the right side.
- The more obstacles there are, the better and more stable the map which is generated through SLAM.
- Obstacles representing cars, which are placed on the right side of the car, need to be aligned on a straight axis. The algorithm used places one straight axis through the obstacles detected and aligns the car in accordance with this. The car thus cannot follow a bending street reliably.
- When running the program on the hardware, walking around the driving space is not recommended. The produced map is less stable, and the position of the car might jump.
