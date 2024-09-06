import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("agv_sim")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    map_subscribe_transient_local = LaunchConfiguration("map_subscribe_transient_local")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "default_bt_xml_filename": default_bt_xml_filename,
        "autostart": autostart,
        "map_subscribe_transient_local": map_subscribe_transient_local,
    }

    # 动态地修改YAML配置文件中的参数，以适应不同的启动需求
    configured_params = RewrittenYaml(
        source_file=params_file,  # 指定了原始的YAML文件路径
        root_key=namespace,  # 命名空间（namespace），所有修改后的参数将被放置在这个命名空间下
        param_rewrites=param_substitutions,  # 字典，包含了需要替换的参数及其新值
        convert_types=True,  # 将尝试将字符串转换为适当的数值类型（如整数、浮点数或布尔值）
    )

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Top-level namespace"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_dir, "config", "nav2_params.yaml"),
                description="Full path to the ROS2 parameters file to use",
            ),
            DeclareLaunchArgument(
                "default_bt_xml_filename",
                default_value=os.path.join(
                    get_package_share_directory("nav2_bt_navigator"),
                    "behavior_trees",
                    "navigate_w_replanning_and_recovery.xml",
                ),
                description="Full path to the behavior tree xml file to use",
            ),
            DeclareLaunchArgument(
                "map_subscribe_transient_local",
                default_value="false",
                description="是否将地图订阅者的QoS设置为瞬态本地,确保订阅者在订阅时能够接收到最新消息",
            ),
            # 局部路径规划和控制器
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            # 全局路径规划器接，计算到达目标的路径。它还托管全局成本图。
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            # 行为插件，用于处理导航中的故障恢复
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            # 行为树导航模块
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            # 生命周期管理器模块实现了以确定性方式处理堆栈的生命周期转换状态的方法。
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
