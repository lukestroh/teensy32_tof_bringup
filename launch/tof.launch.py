#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo
)
from launch.substitutions import (
    LaunchConfiguration
)
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            name="serial_port",
            default_value="/dev/ttyACM0",
            description="Port name for serial device"
        )
    )

    serial_port = LaunchConfiguration("serial_port")

    node_micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="both",
        arguments=[
            "serial",
            "--dev",
            serial_port
        ]
    )

    node_tof_filtered = Node(
        package="teensy32_tof_bringup",
        executable="tof",
        name="tof_filtered",
        output="screen"
    )

    return LaunchDescription(
        declared_args + [
            node_micro_ros_agent,
            node_tof_filtered
        ]
    )
