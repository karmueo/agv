import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from os import path


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "agv_sim"  # <--- CHANGE ME

    # 机器人描述发布
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )

    # 启动gazebo
    world = path.join(
        get_package_share_directory(package_name),
        "worlds",
        "empty.sdf",
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments=[("gz_args", [world, " -r -v ", "3"])],
    )

    # 在gazebo中创建机器人
    ros_gz_sim_create = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-topic",
            "robot_description",
            "--ros-args",
            "--log-level",
            "warn",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory(package_name),
                    "config",
                    "ros_gz_bridge.yaml",
                ),
                "use_sim_time": True,
            }
        ],
    )

    # 把nav2，跟踪，cmd_vel等话题合并成一个话题以控制机器人
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    # 驱动控制器
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            ros_gz_sim_create,
            ros_gz_bridge,
            twist_mux,
            diff_drive_spawner,
            joint_broad_spawner,
        ]
    )
