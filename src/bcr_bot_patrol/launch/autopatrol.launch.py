"""
Author: 沈昌力
Date: 2024-10-09 17:56:14
LastEditTime: 2024-10-09 17:56:37
LastEditors: 沈昌力
Description: 
FilePath: /agv/src/bcr_bot_patrol/launch/autopatrol.launch.py
"""

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取与拼接默认路径
    autopatrol_robot_dir = get_package_share_directory("bcr_bot_patrol")
    patrol_config_path = os.path.join(
        autopatrol_robot_dir, "config", "patrol_config.yaml"
    )

    action_node_control = launch_ros.actions.Node(
        package="bcr_bot_patrol",
        executable="patrol_node",
        parameters=[patrol_config_path],
    )

    action_node_patrol_client = launch_ros.actions.Node(
        package="bcr_bot_patrol",
        executable="speaker",
    )

    return launch.LaunchDescription(
        [
            action_node_control,
            action_node_patrol_client,
        ]
    )
