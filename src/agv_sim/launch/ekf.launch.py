"""
Author: 沈昌力
Date: 2024-09-06 11:25:29
LastEditTime: 2024-09-29 18:20:03
LastEditors: 沈昌力
Description: 
FilePath: /agv/src/agv_sim/launch/ekf.launch.py
"""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("agv_sim"),
                        "config",
                        "ekf.yaml",
                    ),
                    {"use_sim_time": True},
                ],
                # remappings=[
                #     ("/odom", "/diff_cont/odom"),
                # ],
                # remappings=[
                #     ("odometry/filtered", "/odom"),
                # ],
            ),
        ]
    )
