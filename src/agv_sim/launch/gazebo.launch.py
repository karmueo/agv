#!/usr/bin/env python3

from os.path import join


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():
    # Get agv_sim package's share directory path
    agv_sim_path = get_package_share_directory("agv_sim")

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    world_file = LaunchConfiguration(
        "world_file", default=join(agv_sim_path, "worlds", "small_warehouse.sdf")
    )

    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    # spawing agv_sim
    spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory("agv_sim"),
                "launch",
                "agv_sim_gazebo_spawn.launch.py",
            )
        ),
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            AppendEnvironmentVariable(
                name="GAZEBO_MODEL_PATH", value=join(agv_sim_path, "models")
            ),
            SetEnvironmentVariable(
                name="GAZEBO_RESOURCE_PATH",
                value="/usr/share/gazebo-11:" + join(agv_sim_path, "worlds"),
            ),
            DeclareLaunchArgument("world", default_value=world_file),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
            gazebo,
            spawn_node,
        ]
    )
