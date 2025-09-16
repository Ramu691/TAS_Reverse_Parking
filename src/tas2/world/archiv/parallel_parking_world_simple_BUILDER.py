# Builder function nötig um Variablen zu setzen --> beeinflusst die simple Variante mit einer durchgehenden Wand

import os

parking_length = 1.2
parking_width = 0.7
firstWall_length = 3
secondWall_length = 2
walls_heigth = 1
walls_width = 0.2

sdf_content = f"""
<?xml version="1.0" ?>
<sdf version="1.6">

  <world name="parallel_parking_world_simple">
    <!-- Include necessary plugins -->
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

      <model name='my_ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>28 28</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>model://my_ground_plane/materials/scripts</uri>
              <uri>model://my_ground_plane/materials/textures</uri>
              <name>MyGroundPlane/Image</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
        <enable_wind>0</enable_wind>
      </link>
    </model>

    <!-- Wall with parking cutout -->
    <model name="wall_with_parking_cutout">
      <static>true</static>
      <!-- First wall segment -->
      <link name="wall_segment_1">
        <pose>{walls_width/2} {firstWall_length/2} {walls_heigth/2} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{walls_width} {firstWall_length} {walls_heigth}</size> 
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{walls_width} {firstWall_length} {walls_heigth}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
      <!-- Second wall segment -->
      <link name="wall_segment_2">
        <pose>{walls_width/2} {firstWall_length+parking_length+secondWall_length/2} {walls_heigth/2} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{walls_width} {secondWall_length} {walls_heigth}</size> <!-- Length 2m, Width 0.2m, Height 1m -->
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{walls_width} {secondWall_length} {walls_heigth}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
      <!-- Parking space cutout (visual only, no collision for entry) -->
      <link name="pspace_1">
        <pose>{parking_width/2+walls_width} {firstWall_length-walls_width/2} {walls_heigth/2} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{parking_width} {walls_width} {walls_heigth}</size> 
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{parking_width} {walls_width} {walls_heigth}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>

      <link name="pspace_2">
        <pose>{parking_width+walls_width/2} {firstWall_length+parking_length/2} {walls_heigth/2} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{walls_width} {parking_length} {walls_heigth}</size> 
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{walls_width} {parking_length} {walls_heigth}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>

      <link name="pspace_3">
        <pose>{parking_width/2+walls_width} {firstWall_length+parking_length+walls_width/2} {walls_heigth/2} 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{parking_width} {walls_width} {walls_heigth}</size> 
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{parking_width} {walls_width} {walls_heigth}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Physics and other world settings -->
    <physics name="default_physics" default="true" type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
"""

current_directory = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(current_directory, "parallel_parking_world_simple.sdf")

# Write the SDF content to the file
with open(file_path, "w") as file:
    file.write(sdf_content)

