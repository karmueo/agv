"""
Author: 沈昌力
Date: 2024-09-14 16:51:47
LastEditTime: 2024-10-15 14:20:18
LastEditors: 沈昌力
Description: 
FilePath: /agv/src/agv_sim/launch/launch_sim.launch.py
"""

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from moveit_configs_utils import MoveItConfigsBuilder


def generate_demo_launch(moveit_config, launch_package_path=None):
    # 如果没有指定 launch_package_path，就使用 moveit_config.package_path，即 `moveit_config` 的包路径
    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # 如果有虚拟关节，则通过包含 virtual_joints launch 来广播 static tf
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )
    return ld


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    agv_sim_path = get_package_share_directory("agv_sim")
    world_file = LaunchConfiguration(
        "world_file", default=join(agv_sim_path, "worlds", "empty.sdf")
    )
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    spawn_agv_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(agv_sim_path, "launch", "agv_sim_gazebo_spawn.launch.py")
        ),
        launch_arguments={
            # Pass any arguments if your spawn.launch.py requires
        }.items(),
    )

    # 生成 MoveIt 的 launch 文件
    # moveit_config = MoveItConfigsBuilder(
    #     "panda", package_name="my_arm"
    # ).to_moveit_configs()

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH", value=join(agv_sim_path, "worlds")
            ),
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH", value=join(agv_sim_path, "models")
            ),
            DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
            DeclareLaunchArgument("world_file", default_value=world_file),
            gz_sim,
            spawn_agv_sim_node,
            # generate_demo_launch(moveit_config),
        ]
    )
