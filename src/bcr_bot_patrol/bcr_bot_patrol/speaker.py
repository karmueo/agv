#!/usr/bin/env python3
"""
Author: 沈昌力
Date: 2024-10-09 16:58:53
LastEditTime: 2024-10-09 16:58:55
LastEditors: 沈昌力
Description: 
FilePath: /agv/src/bcr_bot_patrol/bcr_bot_patrol/speaker.py
"""

import rclpy
from rclpy.node import Node
from bcr_bot_patrol_interfaces.srv import SpeachText
import espeakng


class Speaker(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # 创建一个服务，服务消息为 SpeachText，服务名为 "speech_text"，回调函数为 speak_text_callback
        self.speech_service = self.create_service(
            SpeachText, "speech_text", self.speak_text_callback
        )
        # 创建一个 espeakng.Speaker 实例
        self.speaker = espeakng.Speaker()
        # 设置语音为中文
        self.speaker.voice = "zh"

    def speak_text_callback(self, request, response):
        self.get_logger().info("正在朗读 %s" % request.text)
        self.speaker.say(request.text)
        self.speaker.wait()
        response.result = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Speaker("speaker")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
