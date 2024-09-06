### 启动示例

```bash
ros2 launch agv launch_sim.launch.py
```

控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_keyboard
```

### 传感器融合

```bash
sudo apt-get install ros-humble-robot-localization
```

参考`src/agv/config/ekf.yaml`