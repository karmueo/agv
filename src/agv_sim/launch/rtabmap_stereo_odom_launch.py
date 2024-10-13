"""
Author: 沈昌力
Date: 2024-09-20 10:24:42
LastEditTime: 2024-09-20 14:27:42
LastEditors: 沈昌力
Description: 
FilePath: /agv/src/agv_sim/launch/rtabmap_stereo_odom_launch.py
"""

# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_sync:=true
#
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
)
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    parameters = [
        {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "subscribe_odom_info": False,
            "approx_sync": False,
            "visual_odometry": False,
            "subscribe_scan": True,
            "use_sim_time": True,
            "rviz": True,
            "scan_topic": "/lidar",
        }
    ]

    remappings = [
        # ("imu", "/imu/data"),
        ("rgb/image", "/camera/rgb/image_rect_color"),
        ("rgb/camera_info", "/camera/rgb/camera_info"),
        ("depth/image", "/camera/depth_registered/image_raw"),
        ("odom", "/diff_cont/odom"),
        ("scan", "/lidar"),
    ]

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rtabmap_launch"),
                "launch",
                "rtabmap.launch.py",
            )
        ),
        launch_arguments={
            "frame_id": "base_link",
            "subscribe_depth": "true",
            "subscribe_odom_info": "false",
            "approx_sync": "false",
            "visual_odometry": "false",
            "subscribe_scan": "true",
            "use_sim_time": "true",
            "scan_topic": "/lidar",
            "odom_topic": "/diff_cont/odom",
        }.items(),
    )

    return LaunchDescription([rtabmap])
