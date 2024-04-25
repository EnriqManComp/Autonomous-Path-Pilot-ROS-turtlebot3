# Autonomous Path Retention System in ROS using turtlebot3 waffle_pi

This project serves as a basic example of how to address the problem of keeping a robot within a path.

## Prerequisites

This project is built on:
* Ubuntu 20.04
* ROS Noetic
* Gazebo 11
* (Optional) A catkin workspace. Instructions for creating one can be found [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

To run this project, you must have cloned the following packages into your workspace:
* Turtlebot3 main packages
```
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
```
* Turtlebot3 Simulations for use in Gazebo. Additional information can be found on the [Turtlebot official site](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
```
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Additionally, you should modify the following launch file within the package: /turtlebot3_simulations/turtlebot3_gazebo/turtlebot3_autorace.launch
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

## How to run the Autonomous Path Retention System Project

1) Copy the files `odometry.py` and `race_walk.py` into the main package of your workspace and apply permissions with the commands:
   ```
   chmod +x odometry.py
   chmod +x race_walk.py
   ```
2) In a terminal, run the following commands:
   ```
   export TURTLEBOT3_MODEL=waffle_pi
   roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
   ```
4) In another terminal, run the command: ``` rosrun <name_of_your_package> odometry.py```
5) In another terminal, run the command: ``` rosrun <name_of_your_package> race_walk.py```

***Note: The intention of this project is to provide a basic framework for solving the problem of keeping the robot within the road. Feel free to fine-tune the controls and modify the approach as needed.***


