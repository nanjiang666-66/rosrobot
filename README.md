# robot_nav_course

课程项目：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11 下的轮式机器人仿真与自主导航。**自编运行节点与后续 A* 算法统一使用 C++；启动描述使用 ROS 2 XML，机器人模型使用 URDF/Xacro，世界使用 SDF。** 旧 Python 源码已移出项目，仅作备份。

## 目录

```text
src/
├── course_bot_description/  # 机器人模型、RViz、XML 模型预览入口
├── course_bot_gazebo/       # 障碍物世界和 XML Gazebo 启动入口
├── course_bot_control/      # C++ 基础运动、A* 路径跟踪和应急停车
├── course_bot_planner/      # 独立 C++ A* 算法、演示程序和测试
├── course_bot_slam/         # SLAM 建图、RViz 点选终点与完整导航启动
└── course_bot_bringup/      # XML 顶层仿真入口
```

第 4 阶段已有独立 C++ A* 算法与测试。规划节点启动时使用 libsdformat 读取 Gazebo 世界中的静态 `box`、`cylinder`、`sphere` 碰撞体，自动完成二维栅格化和车身安全膨胀，不再需要手工同步障碍物坐标。它还会从 `/gazebo/model_states` 读取 `course_bot` 的当前世界位姿，因此节点中断后无需重启 Gazebo即可重新定位和规划。运行时，激光雷达会把静态地图之外的命中点作为临时障碍并触发受影响路径的 A* 重规划，路径跟踪器另有近距离停车保护。完整三终端验收流程见 [控制包说明](src/course_bot_control/README.md)。矩形控制本身**不会避障**。

## 构建与启动

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_bringup simulation.launch.xml
```

规划器默认读取已安装的 `course_obstacles.world`：

```bash
ros2 run course_bot_planner astar_planner --ros-args \
  -p use_sim_time:=true -p goal_x:=4.0 -p goal_y:=3.0
```

如果需要测试另一份世界，可追加 `-p world_file:=/绝对路径/other.world`。加载器会按法向量自动忽略水平无限 `plane`（地面，包括 Gazebo 可能生成的 `ground_plane_0`），但会拒绝当前尚不能安全栅格化的倾斜或竖直 `plane`。只有排查问题时才应使用 `-p map_source:=builtin` 切回内置备用地图；SDF 损坏、其他静态碰撞形状不受支持或位姿无法解析时，节点会明确报错并停止，避免发布漏障碍物的地图。

另开终端并再次 `source`，低速测试 C++ 控制节点：

```bash
ros2 run course_bot_control basic_motion --ros-args \
  -p mode:=straight -p target_distance:=0.3 -p linear_speed:=0.1
```

完整参数、旋转与矩形命令见 [控制包说明](src/course_bot_control/README.md)。不要同时向 `/cmd_vel` 运行多个控制节点。

本项目会调用 ROS 2/Gazebo 官方提供的 `xacro`、`gazebo_ros` 启动器与生成机器人工具；这些外部依赖的实现语言不属于课程项目的自编代码。

## SLAM 建图

`slam-exploration` 分支提供独立建图入口：

```bash
ros2 launch course_bot_slam slam_mapping.launch.xml
```

它不会启动 A* 或路径跟踪器。手动扫描、同时保存占据地图与 SLAM 位姿图的方法见
[`course_bot_slam` 说明](src/course_bot_slam/README.md)。

## 版本管理

稳定功能保存在 `main`，后续 SLAM 探索功能在 `slam-exploration` 分支开发。日常提交、上传 GitHub、查看历史和恢复旧版本的方法见 [Git 与 GitHub 简明使用流程](docs/git-github-workflow.md)。
