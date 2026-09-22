# 第二阶段恢复说明

本次恢复补齐了差速轮式机器人、后万向轮、二维激光雷达、Gazebo 差速驱动插件、激光插件、障碍物世界和统一启动文件。

## 编译

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
source /opt/ros/humble/setup.bash
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
```

项目源码位于 VMware 共享目录，而 `build`、`install` 和 `log` 输出位于
Ubuntu 本机的 `~/robot_nav_course_colcon_cpp`。这是因为 `/mnt/hgfs` 不支持
`colcon --symlink-install` 所需的符号链接。
第三阶段改为 C++ 后，构建脚本使用新的输出目录，旧的 Python 构建产物仍保留。

## 启动仿真

```bash
ros2 launch course_bot_bringup simulation.launch.xml
```

## 控制验证

新开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run course_bot_control basic_motion --ros-args \
  -p mode:=straight -p target_distance:=0.3 -p linear_speed:=0.1
```

控制节点会按里程计定距停车；按 `Ctrl+C` 退出时也会发送零速度。不要只用非零速度的 `ros2 topic pub` 测试后直接中断：Gazebo 差速插件可能保留最后一条速度命令。检查激光与里程计：

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```
