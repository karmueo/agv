#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc


def generate_launch_description():

    agv_sim_path = get_package_share_directory("agv_sim")
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        join(agv_sim_path, "description/robot.urdf.xacro"),
                        " camera_enabled:=",
                        camera_enabled,
                        " stereo_camera_enabled:=",
                        stereo_camera_enabled,
                        " two_d_lidar_enabled:=",
                        two_d_lidar_enabled,
                        " sim_ign:=",
                        "true",
                    ]
                )
            }
        ],
        # remappings=[
        #     ("/joint_states", "agv_sim/joint_states"),
        # ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "agv_sim",
            "-allow_renaming",
            "true",
            "-z",
            "0.28",
            "-x",
            position_x,
            "-y",
            position_y,
            "-Y",
            orientation_yaw,
        ],
    )

    # 驱动控制器
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/kinect_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/kinect_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "stereo_camera/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/kinect_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # "/world/default/model/agv_sim/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
        ],
        remappings=[
            # ("/world/default/model/agv_sim/joint_state", "agv_sim/joint_states"),
            # ("/odom", "agv_sim/odom"),
            ("/scan", "/agv_sim/scan"),
            ("/kinect_camera/image", "agv_sim/kinect_camera/image"),
            ("/kinect_camera/depth_image", "agv_sim/kinect_camera/depth_image"),
            ("/stereo_camera/left/image_raw", "agv_sim/stereo_camera/left/image_raw"),
            ("/stereo_camera/right/image_raw", "agv_sim/stereo_camera/right/image_raw"),
            ("/imu", "agv_sim/imu"),
            # ("/cmd_vel", "agv_sim/cmd_vel"),
            ("kinect_camera/camera_info", "agv_sim/kinect_camera/camera_info"),
            (
                "stereo_camera/left/camera_info",
                "agv_sim/stereo_camera/left/camera_info",
            ),
            (
                "stereo_camera/right/camera_info",
                "agv_sim/stereo_camera/right/camera_info",
            ),
            ("/kinect_camera/points", "agv_sim/kinect_camera/points"),
        ],
    )

    # transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "--x",
    #         "0.0",
    #         "--y",
    #         "0.0",
    #         "--z",
    #         "0.0",
    #         "--yaw",
    #         "0.0",
    #         "--pitch",
    #         "0.0",
    #         "--roll",
    #         "0.0",
    #         "--frame-id",
    #         "kinect_camera",
    #         "--child-frame-id",
    #         "agv_sim/base_footprint/camera",
    #     ],
    # )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "kinect_camera",
            "agv_sim/base_footprint/camera",
        ],
    )

    # robot_localization_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[
    #         join(agv_sim_path, "config/ekf.yaml"),
    #         {"use_sim_time": LaunchConfiguration("use_sim_time")},
    #     ],
    #     remappings=[
    #         ("odometry/filtered", "/odom"),
    #     ],
    # )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_enabled", default_value=camera_enabled),
            DeclareLaunchArgument(
                "stereo_camera_enabled", default_value=stereo_camera_enabled
            ),
            DeclareLaunchArgument(
                "two_d_lidar_enabled", default_value=two_d_lidar_enabled
            ),
            DeclareLaunchArgument("position_x", default_value="0.0"),
            DeclareLaunchArgument("position_y", default_value="0.0"),
            DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
            DeclareLaunchArgument("odometry_source", default_value="world"),
            robot_state_publisher,
            gz_spawn_entity,
            transform_publisher,
            gz_ros2_bridge,
            diff_drive_spawner,
            joint_broad_spawner,
            # robot_localization_node,
        ]
    )
