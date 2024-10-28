<!--
 * @Author: 沈昌力
 * @Date: 2024-09-06 10:29:35
 * @LastEditTime: 2024-10-14 09:12:06
 * @LastEditors: 沈昌力
 * @Description: 
 * @FilePath: /agv/README.md
-->
# Project Title

## Table of Contents

- [Project Title](#project-title)
  - [Table of Contents](#table-of-contents)
  - [About ](#about-)
  - [Getting Started ](#getting-started-)
    - [安装依赖](#安装依赖)
    - [Building](#building)
    - [启动示例](#启动示例)

## About <a name = "about"></a>

本项目是基于ros2 humble和Gazebo Fortress的agv仿真。

## Getting Started <a name = "getting_started"></a>

### 安装依赖

```bash
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt install ros-$ROS_DISTRO-ros-gz
sudo apt install ros-$ROS_DISTRO-ign-ros2-control
```

### Building

```bash
# Install dependencies
IGNITION_VERSION=fortress rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .

# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```


### 启动示例

```bash
source install/local_setup.bash
```

1. AGV+机械臂仿真启动
```bash
# 通过参数arm_enabled来添加或者不添加机械臂
ros2 launch agv_with_arm gazebo.launch.py arm_enabled:=true
```

可以通过下面的命令启动键盘控制来控制AGV
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

2. 单独机械臂仿真启动
```bash
ros2 launch agv_with_arm arm_gazebo.py
```

3. 机械臂moveit2规划和控制
```bash
ros2 launch agv_with_arm moveit2_group.launch.py
```