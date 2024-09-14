import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "agv_sim"  # <--- CHANGE ME
    bringup_dir = get_package_share_directory(package_name)

    # Create the launch configuration variables
    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="是否将命名空间应用于导航堆栈",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="是否运行slam"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            bringup_dir,
            "maps",
            "my_map.yaml",
        ),
        description="建图得到的yaml文件全路径",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
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
        description="是否使用组合启动",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="如果节点崩溃是否重生。当组合被禁用时应用",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "config", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "launch_sim.launch.py",
                )
            ]
        )
    )

    launch_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "sensor.launch.py",
                )
            ]
        )
    )

    # TODO: 3.运行 Rviz 并加载默认配置以查看 move_group 节点的状态
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_name), "config", "rviz_config.rviz"
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    # 设置 TimerAction，这里设置等待时间为 10 秒
    delay_rviz2_start = TimerAction(
        period=5.0,  # 延迟时间，单位为秒
        actions=[rviz2],  # 延时完成后要执行的动作，即启动 rviz2
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "bring_up.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    # Launch them all!
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(launch_sim)
    ld.add_action(launch_sensor)
    # ld.add_action(bringup_cmd)
    ld.add_action(delay_rviz2_start)

    return ld
    # return LaunchDescription([launch_sim, launch_sensor, delay_rviz2_start])
