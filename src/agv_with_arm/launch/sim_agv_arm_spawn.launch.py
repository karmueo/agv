#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
import yaml
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


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

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_path,
                " arm_enabled:=true",  # 传递参数 arm_enabled
            ]
        )
    }

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
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
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_drive_controller"],
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    # )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["aubo_controller", "-c", "/controller_manager"],
    )

    # ***** STATIC TRANSFORM ***** #
    # NODE -> Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "arm_base_link",
            "roof_link",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("position_x", default_value="0.0"),
            DeclareLaunchArgument("position_y", default_value="0.0"),
            DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
            robot_state_publisher,
            spawn_entity,
            static_tf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[
                        joint_broad_spawner,
                    ],
                )
            ),
            # RegisterEventHandler(
            #     OnProcessExit(
            #         target_action=joint_broad_spawner,
            #         on_exit=[
            #             diff_drive_spawner,
            #         ],
            #     )
            # ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_broad_spawner,
                    on_exit=[
                        joint_trajectory_controller_spawner,
                    ],
                )
            ),
        ]
    )
