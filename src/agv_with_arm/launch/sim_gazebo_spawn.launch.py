#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node


def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc


def generate_launch_description():
    # Get package's share directory path
    this_package_path = get_package_share_directory("agv_with_arm")

    # Retrieve launch configuration arguments
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")

    # Path to the Xacro file
    xacro_path = join(this_package_path, "urdf", "agv", "robot.urdf.xacro")

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        xacro_path,
                    ]
                )
            }
        ],
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "agv_sim_bot",
            "-z",
            "0.28",
            "-x",
            position_x,
            "-y",
            position_y,
            "-z",
            "0.2",
            "-Y",
            orientation_yaw,
        ],
    )

    # 驱动控制器
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("position_x", default_value="0.0"),
            DeclareLaunchArgument("position_y", default_value="0.0"),
            DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
            robot_state_publisher,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
        ]
    )
