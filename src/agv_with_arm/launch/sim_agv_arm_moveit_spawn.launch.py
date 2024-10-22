#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
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

    # *********************** MoveIt!2 *********************** #
    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    robot_description_semantic_config = load_file("agv_with_arm", "config/aubo.srdf")
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("agv_with_arm", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("agv_with_arm", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml(
        "agv_with_arm", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    # Command-line argument: RVIZ file?
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )
    # RVIZ:
    rviz_base = join(get_package_share_directory("agv_with_arm"), "config")
    rviz_full_config = join(rviz_base, "agv_with_arm.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {"use_sim_time": True},
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
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_broad_spawner,
                    on_exit=[
                        diff_drive_spawner,
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
                        # MoveIt!2:
                        TimerAction(
                            period=5.0,
                            actions=[rviz_arg, rviz_node_full, run_move_group_node],
                        ),
                    ],
                )
            ),
        ]
    )
