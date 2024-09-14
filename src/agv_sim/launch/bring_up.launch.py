import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("agv_sim")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")  # 是否启用建图，默认为True
    map_yaml_file = LaunchConfiguration("map")  # 默认加载的地图
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # use_sim_time默认为True
    # map_yaml_file默认为maps/my_map.yaml
    param_substitutions = {"use_sim_time": use_sim_time}

    params_file = ReplaceString(
        source_file=params_file,  # 默认为nav2_raw_params.yaml
        replacements={
            "<robot_namespace>": ("/", namespace)
        },  # 把<robot_namespace>替换为namespace
        condition=IfCondition(use_namespace),  # 如果use_namespace为True，则执行替换
    )

    # 根据需要进行参数替换和类型转换
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,  # 默认为nav2_raw_params.yaml
            root_key=namespace,  # root_key为namespace，默认为/
            param_rewrites=param_substitutions,  # 参数替换见param_substitutions
            convert_types=True,  # 自动把参数转换为正确的类型
        ),
        allow_substs=True,  # 允许参数替换
    )

    # 设置环境变量，以便立即将消息打印到stdout
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="True", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value="",  # 默认加载的地图
        description="Full path to map yaml file to load",
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
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                output="screen",
            ),
            # 如果设置了slam，则启动slam_toolbox
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "slam_launch.py")
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "params_file": params_file,
                }.items(),
            ),
            # 如果没有设置slam，则启动localization_launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization_launch.py")
                ),
                condition=IfCondition(PythonExpression(["not ", slam])),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, "navigation_launch.py")
            #     ),
            #     launch_arguments={
            #         "namespace": namespace,
            #         "use_sim_time": use_sim_time,
            #         "autostart": autostart,
            #         "params_file": params_file,
            #         "use_composition": use_composition,
            #         "use_respawn": use_respawn,
            #         "container_name": "nav2_container",
            #         "default_bt_xml_filename": os.path.join(
            #             launch_dir, "config", "fllow_point.xml"
            #         ),
            #     }.items(),
            # ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
