# Project Title

## Table of Contents

- [Project Title](#project-title)
  - [Table of Contents](#table-of-contents)
  - [About ](#about-)
  - [Getting Started ](#getting-started-)
    - [安装依赖](#安装依赖)
    - [Building](#building)
    - [URDF转SDF](#urdf转sdf)
    - [](#)
    - [启动示例](#启动示例)

## About <a name = "about"></a>

Write about 1-2 paragraphs describing the purpose of your project.

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

### URDF转SDF

```bash
ros2 run aubo_description xacro2sdf.bash
```

### 

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/path/to/your/models
```

### 启动示例

```bash
source install/local_setup.bash

ros2 launch moveit_assistant ign_moveit.launch.py
```
