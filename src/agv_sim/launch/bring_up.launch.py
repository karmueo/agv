import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "agv_sim"  # <--- CHANGE ME

    # 启动机器人仿真
    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "launch_robot.launch.py",
                )
            ]
        ),
    )

    # 启动建图
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "online_async_launch.py",
                )
            ]
        ),
    )

    delay_slam_start = TimerAction(
        period=5.0,  # 延迟时间，单位为秒
        actions=[slam],  # 延时完成后要执行的动作
    )

    # Launch them all!
    return LaunchDescription([robot_sim, delay_slam_start])
