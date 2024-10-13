from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from bcr_bot_patrol_interfaces.srv import SpeachText
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# 继承自 BasicNavigator 类，BasicNavigator实现了导航相关的基本功能，比如导航到指定点、获取当前位姿等
class PatrolNode(BasicNavigator):
    def __init__(self, node_name="patrol_node"):
        super().__init__(node_name)
        # 声明参数 "target_points"，默认值为 [0.0, 0.0, 0.0]
        self.declare_parameter("target_points", [0.0, 0.0, 0.0])
        # 声明参数 "initial_point"，默认值为 [0.0, 0.0, 0.0]
        self.declare_parameter("initial_point", [0.0, 0.0, 0.0])
        # 获取参数 "target_points" 的值
        self.target_points = self.get_parameter("target_points").value
        # 获取参数 "initial_point" 的值
        self.initial_point = self.get_parameter("initial_point").value
        # 实时位置获取 TF 相关定义
        # 创建一个TF缓存对象，用于存储坐标变换
        self.buffer = Buffer()
        # 创建一个TF监听器对象，用于监听坐标变换
        self.listener = TransformListener(self.buffer, self)
        # 语音合成客户端
        self.speach_client = self.create_client(SpeachText, "speech_text")
        # 订阅与保存图像相关定义
        # 声明参数 "image_save_path"，默认值为空字符串
        self.declare_parameter("image_save_path", "")
        # 获取参数 "image_save_path" 的值
        self.image_save_path = self.get_parameter("image_save_path").value
        # 创建 CvBridge 对象，用于图像转换
        self.bridge = CvBridge()
        # 初始化最新图像变量
        self.latest_image = None
        # 创建图像订阅，订阅摄像头图像话题，回调函数为 image_callback，队列长度为 10
        self.subscription_image = self.create_subscription(
            Image, "/bcr_bot/kinect_camera/image", self.image_callback, 10
        )

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过 x,y,yaw 合成 PoseStamped
        """
        pose = PoseStamped()  # 创建一个PoseStamped对象
        pose.header.frame_id = "map"  # 设置坐标系为"map"
        pose.pose.position.x = x  # 设置位置的x坐标
        pose.pose.position.y = y  # 设置位置的y坐标
        rotation_quat = quaternion_from_euler(0, 0, yaw)  # 将yaw角转换为四元数
        pose.pose.orientation.x = rotation_quat[0]  # 设置四元数的x分量
        pose.pose.orientation.y = rotation_quat[1]  # 设置四元数的y分量
        pose.pose.orientation.z = rotation_quat[2]  # 设置四元数的z分量
        pose.pose.orientation.w = rotation_quat[3]  # 设置四元数的w分量
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        # 从参数获取初始化点
        self.initial_point = self.get_parameter("initial_point").value
        # 合成位姿并进行初始化
        self.setInitialPose(
            self.get_pose_by_xyyaw(
                self.initial_point[0], self.initial_point[1], self.initial_point[2]
            )
        )
        # 等待直到导航激活
        self.waitUntilNav2Active()

    def get_target_points(self):
        """
        通过参数值获取目标点集合
        """
        points = []
        # 从参数获取目标点集合
        self.target_points = self.get_parameter("target_points").value
        for index in range(
            int(len(self.target_points) / 3)
        ):  # 遍历目标点集合，每三个元素为一组
            x = self.target_points[index * 3]  # 获取目标点的x坐标
            y = self.target_points[index * 3 + 1]  # 获取目标点的y坐标
            yaw = self.target_points[index * 3 + 2]  # 获取目标点的yaw角
            points.append([x, y, yaw])  # 将目标点添加到points列表中
            self.get_logger().info(
                f"获取到目标点: {index}->({x},{y},{yaw})"
            )  # 记录获取到的目标点信息
        return points

    def nav_to_pose(self, target_pose):
        """
        导航到指定位姿
        """
        self.waitUntilNav2Active()  # 等待导航系统激活
        result = self.goToPose(target_pose)  # 导航到目标位姿
        while not self.isTaskComplete():  # 循环直到任务完成
            feedback = self.getFeedback()  # 获取导航反馈
            if feedback:  # 如果有反馈信息
                self.get_logger().info(
                    f"剩余距离: {feedback.distance_remaining} m"
                )  # 记录预计到达时间
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("导航结果：成功")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("导航结果：被取消")
        elif result == TaskResult.FAILED:
            self.get_logger().error("导航结果：失败")
        else:
            self.get_logger().error("导航结果：返回状态无效")

    def get_current_pose(self):
        """
        通过TF获取当前位姿
        """
        while rclpy.ok():  # 当ROS2系统正常运行时
            try:
                # 获取从"map"到"base_footprint"的坐标变换
                tf = self.buffer.lookup_transform(
                    "map",  # 目标坐标系
                    "base_footprint",  # 源坐标系
                    rclpy.time.Time(seconds=0),  # 查询最新的变换
                    rclpy.time.Duration(seconds=1),  # 超时时间为1秒
                )
                transform = tf.transform  # 获取变换信息
                rotation_euler = euler_from_quaternion(
                    [
                        transform.rotation.x,  # 四元数的x分量
                        transform.rotation.y,  # 四元数的y分量
                        transform.rotation.z,  # 四元数的z分量
                        transform.rotation.w,  # 四元数的w分量
                    ]
                )  # 将四元数转换为欧拉角
                self.get_logger().info(
                    f"平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}"
                )
                return transform
            except Exception as e:
                self.get_logger().warn(f"不能够获取坐标变换，原因: {str(e)}")

    def image_callback(self, msg):
        # msg的类型为sensor_msgs/Image
        self.latest_image = msg

    def record_image(self):
        """
        记录图像
        """
        pose = self.get_current_pose()  # 获取当前位姿
        cv_image = self.bridge.imgmsg_to_cv2(
            self.latest_image
        )  # 将ROS图像消息转换为OpenCV图像
        cv2.imwrite(
            f"{self.image_save_path}image_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png",
            cv_image,
        )  # 将图像保存到指定路径，文件名包含位姿的x和y坐标

    def speach_text(self, text):
        """
        调用服务播放语音
        """
        self.get_logger().info(f"语音合成成功：{text}")
        # while not self.speach_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("语音课程服务未上线，等待中。。。")

        # request = SpeachText.Request()
        # request.text = text
        # future = self.speach_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is not None:
        #     result = future.result().result
        #     if result:
        #         self.get_logger().info(f"语音合成成功：{text}")
        #     else:
        #         self.get_logger().warn(f"语音合成失败：{text}")
        # else:
        #     self.get_logger().warn("语音合成服务请求失败")


def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speach_text(text="正在初始化位置")
    patrol.init_robot_pose()
    patrol.speach_text(text="位置初始化完成")

    while rclpy.ok():  # 当ROS2系统正常运行时
        for point in patrol.get_target_points():  # 遍历所有目标点
            x, y, yaw = point[0], point[1], point[2]  # 获取目标点的x, y, yaw坐标
            # 导航到目标点
            target_pose = patrol.get_pose_by_xyyaw(
                x, y, yaw
            )  # 通过x, y, yaw生成目标位姿
            patrol.speach_text(
                text=f"准备前往目标点{x},{y}"
            )  # 播报准备前往目标点的语音
            patrol.nav_to_pose(target_pose)  # 导航到目标位姿
            # 记录图像
            patrol.speach_text(text=f"已到达目标点{x},{y},准备记录图像")
            patrol.record_image()
            patrol.speach_text(text="图像记录完成")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
