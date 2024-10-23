"""
Author: 沈昌力
Date: 2024-09-24 10:37:33
LastEditTime: 2024-10-09 16:19:02
LastEditors: 沈昌力
Description: 
FilePath: /agv/src/bcr_bot/launch/mapping.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    this_package = get_package_share_directory("agv_with_arm")

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    autostart = LaunchConfiguration("autostart", default="True")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start the slam_toolbox stack",
    )

    # Include slam_toolbox launch file
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": os.path.join(
                this_package, "config", "mapper_params_online_async.yaml"
            ),
        }.items(),
    )

    # Launch RViz
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(this_package, "config", "map.rviz")],
    )

    # Static transform publisher
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(slam_toolbox_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(static_transform_publisher_node)

    return ld
