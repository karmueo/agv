import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("agv_sim")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    follow_point_xml_path = LaunchConfiguration("follow_point_xml_path")
    through_poses_xml_path = LaunchConfiguration("through_poses_xml_path")

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # 配置参数替换
    param_substitutions = {
        "use_sim_time": use_sim_time,  # 默认为true
        "autostart": autostart,  # 默认为true
        "default_nav_to_pose_bt_xml": follow_point_xml_path,  # 默认为follow_point.xml
        "default_nav_through_poses_bt_xml": through_poses_xml_path,  # 默认为navigate_through_poses_w_replanning_and_recovery.xml
    }

    # 更新YAML配置文件中的参数
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,  # 默认为nav2_raw_params.yaml
            root_key=namespace,  # 默认为空
            param_rewrites=param_substitutions,  # 见上面
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 设置环境变量，为了确保日志输出是缓冲的，从而提高日志记录的性能或确保日志的完整性
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "nav2_raw_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of conatiner that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="warn", description="log level"
    )

    # Nav2 Behavior Trees to use based on the specified mode
    declare_nav2_bt_through_poses_xml_path = DeclareLaunchArgument(
        "through_poses_xml_path",
        default_value=os.path.join(
            bringup_dir,
            "config",
            "navigate_through_poses_w_replanning_and_recovery.xml",
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_nav2_follow_point_xml_path = DeclareLaunchArgument(
        "follow_point_xml_path",
        default_value=os.path.join(
            bringup_dir,
            "config",
            "follow_point.xml",
        ),
        description="Full path to the behavior tree xml file to use",
    )

    # declare_nav2_bt_to_pose_xml_path = DeclareLaunchArgument(
    #     "to_pose_xml_path",
    #     default_value=os.path.join(
    #         bringup_dir,
    #         "config",
    #         "navigate_to_pose_w_replanning_and_recovery.xml",
    #     ),
    #     description="Full path to the behavior tree xml file to use",
    # )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings
                + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=[configured_params],
                remappings=remappings
                + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_nav2_follow_point_xml_path)
    ld.add_action(declare_nav2_bt_through_poses_xml_path)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
