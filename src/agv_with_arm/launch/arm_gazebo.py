#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get agv_with_arm package's share directory path
    this_package_path = get_package_share_directory("agv_with_arm")

    world_file = LaunchConfiguration(
        "world_file",
        default=join(
            get_package_share_directory("agv_with_arm"), "worlds", "small_warehouse.sdf"
        ),
    )

    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    # Retrieve launch configuration arguments
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")

    # Path to the Xacro file
    xacro_path = join(this_package_path, "urdf", "arm", "robot.urdf.xacro")

    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {"robot_description": robot_description_config}

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
            "aubo_arm",
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
        arguments=["arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    handleft_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["handleft_controller", "-c", "/controller_manager"],
    )
    handright_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["handright_controller", "-c", "/controller_manager"],
    )

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
            "world",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("position_x", default_value="0.0"),
            DeclareLaunchArgument("position_y", default_value="0.0"),
            DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
            # Declare launch arguments
            AppendEnvironmentVariable(
                name="GAZEBO_MODEL_PATH", value=join(this_package_path, "models")
            ),
            SetEnvironmentVariable(
                name="GAZEBO_RESOURCE_PATH",
                value="/usr/share/gazebo-11:"
                + join(get_package_share_directory("agv_with_arm"), "worlds"),
            ),
            DeclareLaunchArgument("world", default_value=world_file),
            gazebo,
            robot_state_publisher,
            spawn_entity,
            # static_tf,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[
                        joint_broad_spawner,
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_broad_spawner,
                    on_exit=[
                        joint_trajectory_controller_spawner,
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_trajectory_controller_spawner,
                    on_exit=[
                        handleft_controller_spawner,
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_trajectory_controller_spawner,
                    on_exit=[
                        handright_controller_spawner,
                    ],
                )
            ),
        ]
    )
