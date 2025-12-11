# Drone-Simulation-with-ROS2-and-PX4
This repository contains a simulation setup for an autonomous multirotor drone using ROS 2, PX4 Autopilot, and Gazebo/Ignition.


first clone this repo into your workspace then run the command

ros2 launch px4_bringup x500.launch.py

then launch the keyteleop launch file by the command 

ros2 launch px4_offboard keyboard_teleop.launch.py

and control the drone using W/A/S/D and arrows keys 

if you don't want to use the QGC then set param NAV_DLL_ACT 0 by using the command

param set NAV_DLL_ACT 0
