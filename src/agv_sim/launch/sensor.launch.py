#!/usr/bin/env -S ros2 launch
"""Visualisation of SDF model for panda in Ignition Gazebo. Note that the generated model://panda/model.sdf descriptor is used."""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_name = "agv_sim"

    nodes = []
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory(package_name),
                    "config",
                    "ros_gz_bridge_sensor.yaml",
                ),
            }
        ],
    )

    nodes.append(ros_gz_bridge)

    return LaunchDescription(nodes)
