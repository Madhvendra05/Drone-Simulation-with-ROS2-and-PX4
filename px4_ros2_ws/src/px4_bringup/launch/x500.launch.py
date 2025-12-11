#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    qgc_path = os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--title=PX4 SITL',
                '--',
                'bash', '-c',
                f'cd {px4_dir} && make px4_sitl gz_x500; exec bash'
            ],
            output='screen',
            name='px4_sitl'
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--title=MicroXRCE Agent',
                        '--',
                        'bash', '-c',
                        'MicroXRCEAgent udp4 -p 8888; exec bash'
                    ],
                    output='screen',
                    name='microxrce_agent'
                )
            ]
        ),

        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=[qgc_path],
        #             output='screen',
        #             name='qgroundcontrol'
        #         )
        #     ]
        # ),

    ])
