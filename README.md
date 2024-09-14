# Project Title

## Table of Contents

- [Project Title](#project-title)
  - [Table of Contents](#table-of-contents)
  - [About ](#about-)
  - [Getting Started ](#getting-started-)
    - [安装依赖](#安装依赖)
    - [Building](#building)
    - [启动示例](#启动示例)
    - [启动键盘控制（可选）](#启动键盘控制可选)
    - [建图](#建图)
      - [加载之前的地图](#加载之前的地图)
    - [定位](#定位)
    - [导航](#导航)

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

下面的命令会启动ign gazebo仿真和rviz显示

```bash

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:${PWD}/install/share

source install/local_setup.bash

ros2 launch agv_sim launch_robot.launch.py
```

### 启动键盘控制（可选）

如过需要键盘控制则执行，在键盘控制过程中，会暂时中止导航，结束控制后继续恢复导航

```bash
控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_keyboard
```
### 建图

```bash
ros2 launch agv_sim bring_up.launch.py
```

#### 加载之前的地图

在rviz中使用SlamToolboxPlugin插件(可能需要自己编译slam_toolbox)。在Deserialize Map输入`install/share/agv_sim/maps/my_map`，然后点击`Deserialize Map`按钮。
可以通过点击`2D Pose Estimate`然后选择`Start At Pose Est.`，再点击`Deserialize Map`来实现位置和地图的匹配。

### 定位

1. 启动定位程序
```bash
ros2 launch agv_sim bring_up.launch.py slam:=False map:=/home/xvshuo/work/scl/agv/install/share/agv_sim/maps/my_map.yaml
```
其中`/home/xvshuo/work/scl/agv/install/share/agv_sim/maps/my_map.yaml`替换会实际的地图。

2. 点击`2D Pose Estimate`然后在地图上设置大概的位置。

### 导航

```bash
ros2 launch agv_sim nav_launch.py
```