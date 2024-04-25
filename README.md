# Road-Lane-Detection-ROS-turtlebot3
Road lane detection using ROS and turtlebot3 

## Prerequirements

This project is built in:
* Ubuntu 20.04
* ROS Noetic
* Gazebo 11
* (optional) A catkin workspace. You can find how to create it [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

You must have clone the following packages in your workspace to run this project:
* Turtlebot3 main packages
```
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
```
* Turtlebot3 Simulations to run in Gazebo.
```
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

You can also find additional information in the [Turtlebot official site](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

Additionally you should change the following launch file: in the package /turtlebot3_simulations/turtlebot3_gazebo/turtlebot3_autorace.launch
```
<launch>
  <env name="GAZEBO_RESOURCE_PATH" value="$(find turtlebot3_gazebo)/models/turtlebot3_autorace/ground_picture" />

  <arg name="x_pos" default="0.245"/>
  <arg name="y_pos" default="-1.787"/>
  <arg name="z_pos" default="0"/>  

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_autorace.world" />
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>  

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_waffle_pi.urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_waffle_pi -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
   
</launch>
```

## How to run the road lane project

1) Copy the files odometry.py and race_walk.py into your main package of the workspace and apply the permissions with the command ```chmod +x odometry.py``` and ```chmod +x race_walk.py```
2) Run in a terminal the following commands:
   ```
   export TURTLEBOT3_MODEL=waffle_pi
   roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
   ```
3) Run in other terminal the command ``` rosrun <name_of_your_package> odometry.py```
4) Run in other terminal the command ``` rosrun <name_of_your_package> race_walk.py```

Note to the disclaimers: The intention of this project is to provide the base framework to the problem of keep the robot inside the road. Feel free to continue the smooth of the controls and modify the solution approach.


