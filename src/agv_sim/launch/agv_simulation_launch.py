# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("agv_sim")
    launch_dir = os.path.join(bringup_dir, "launch")

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

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_simulator = LaunchConfiguration("use_simulator")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

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
        default_value=os.path.join(bringup_dir, "config", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
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

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator",
        default_value="True",
        description="是否启用仿真",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="是否启用robot_state_publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless", default_value="True", description="是否启用仿真gzclient"
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(bringup_dir, "worlds", "empty.sdf"),
        description="Full path to world model file to load",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="agv", description="name of the robot"
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        cwd=[launch_dir],
        output="screen",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless])),
        cmd=["gzclient"],
        cwd=[launch_dir],
        output="screen",
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
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

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld
