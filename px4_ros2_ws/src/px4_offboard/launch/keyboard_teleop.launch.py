#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    package_name = 'px4_offboard'
    ros_distro = 'humble'
    
    ros_setup_path = f'/opt/ros/{ros_distro}/setup.bash'

    controller_node = Node(
        package=package_name,
        executable='velocity_offboard_control.py',
        name='velocity_offboard_control',
        output='screen',
        emulate_tty=True,
    )
    
    teleop_node_process = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            f'source {ros_setup_path} && '
            f'echo "Terminal for Keyboard Teleop (Press Space to Arm)" && '
            f'ros2 run {package_name} teleop_keyboard.py; '
            'exec bash'
        ],
        output='screen',
        shell=False
    )

    return LaunchDescription([
        controller_node,
        teleop_node_process
    ])