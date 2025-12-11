# Drone-Simulation-with-ROS2-and-PX4
This repository contains a simulation setup for an autonomous multirotor drone using ROS 2, PX4 Autopilot, and Gazebo/Ignition.


first clone this repo into your workspace then clone the px4_msgs by using the command 
git clone https://github.com/PX4/px4_msgs.git -b release/1.15

and run the command

git clone https://github.com/PX4/px4_ros_com.git in your directory

build your workspace run the command

ros2 launch px4_bringup x500.launch.py

then run the launch file of sensor_combined_listener.launch.py using the command 

ros2 launch px4_ros_com sensor_combined_listener.launch.py

then launch the keyteleop launch file by the command 

ros2 launch px4_offboard keyboard_teleop.launch.py

and control the drone using W/A/S/D and arrows keys 

if you don't want to use the QGC then set param NAV_DLL_ACT 0 by using the command

param set NAV_DLL_ACT 0
