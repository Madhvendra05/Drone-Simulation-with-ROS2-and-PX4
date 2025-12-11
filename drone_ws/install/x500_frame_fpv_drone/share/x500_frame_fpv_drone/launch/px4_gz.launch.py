import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --------------------------------------------------------------------------
    # PATHS
    # --------------------------------------------------------------------------
    pkg_drone = get_package_share_directory("x500_frame_fpv_drone")

    px4_dir = os.path.expanduser("~/PX4-Autopilot")
    px4_exec = os.path.join(px4_dir, "build/px4_sitl_default/bin/px4")
    rcS_path = os.path.join(px4_dir, "ROMFS/px4fmu_common/init.d-posix/rcS")

    # --------------------------------------------------------------------------
    # LAUNCH ARGUMENTS
    # --------------------------------------------------------------------------
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_drone, "urdf", "x500_frame_fpv_drone.xacro"),
        description="Xacro robot model"
    )

    # --------------------------------------------------------------------------
    # ENVIRONMENT FOR GZ-SIM (Ignition/Garden)
    # --------------------------------------------------------------------------
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(pkg_drone).parent.resolve())
    )

    gz_sim_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=os.path.join(px4_dir, "build/px4_sitl_default/lib")
    )

    # --------------------------------------------------------------------------
    # ROBOT DESCRIPTION (for rviz or future nodes, optional)
    # --------------------------------------------------------------------------
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=True",
            " is_sim:=true",
        ]),
        value_type=str
    )

    # --------------------------------------------------------------------------
    # GZ SIM LAUNCH
    # --------------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments=[
            ("gz_args", "-v 4 -r empty.sdf")
        ]
    )

    # --------------------------------------------------------------------------
    # SPAWN SDF MODEL IN GZ-SIM
    # --------------------------------------------------------------------------
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file", os.path.join(pkg_drone, "models", "x500_frame_fpv_drone.sdf"),
            "-name", "x500_frame_fpv_drone"
        ],
    )

    # --------------------------------------------------------------------------
    # PX4 SITL START (Important: cwd MUST be px4_dir)
    # --------------------------------------------------------------------------
    px4_sitl = ExecuteProcess(
        cmd=[
            px4_exec,
            rcS_path
        ],
        cwd=px4_dir,
        output="screen",
        name="px4_sitl"
    )

    # --------------------------------------------------------------------------
    # MICRO XRCE DDS AGENT (PX4 ↔ ROS 2 communication)
    # --------------------------------------------------------------------------
    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="microxrce_agent"
    )

    # --------------------------------------------------------------------------
    # ROS ↔ GZ BRIDGE (IMU, clock, cmd_vel → PX4)
    # --------------------------------------------------------------------------
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist",
        ],
        remappings=[
            ("/imu", "/imu/out"),
            ("/cmd_vel", "/drone/cmd_vel"),
        ]
    )

    # --------------------------------------------------------------------------
    # FINAL LAUNCH DESCRIPTION
    # --------------------------------------------------------------------------
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        gz_sim_plugin_path,
        gazebo,
        gz_spawn_entity,
        px4_sitl,
        microxrce_agent,
        gz_ros2_bridge,
    ])
