import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "agv_sim"  # <--- CHANGE ME

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
        arguments=["-d", rviz_config_dir],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    # 设置 TimerAction，这里设置等待时间为 10 秒
    delay_rviz2_start = TimerAction(
        period=5.0,  # 延迟时间，单位为秒
        actions=[rviz2],  # 延时完成后要执行的动作，即启动 rviz2
    )

    # Launch them all!
    return LaunchDescription([launch_sim, launch_sensor, delay_rviz2_start])
