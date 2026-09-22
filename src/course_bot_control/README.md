# 第三阶段：C++ 运动控制

本包只构建 C++ 节点；不依赖旧 Python 控制模块。`src/basic_motion.cpp` 用 `/odom` 反馈控制 `/cmd_vel`，支持定距直行、原地左转约 90° 和矩形轨迹。`src/path_follower.cpp` 沿 A* 路径低速行驶；`src/stop_robot.cpp` 用于异常退出后的应急停车。旧 Python 代码已移到项目外备份，仅供对照。

## 阅读代码

- `on_odom()`：读取里程计位置与四元数，计算朝向和转弯累计角度。
- `control_step()`：每 0.1 秒检查里程计、车体倾斜和动作超时，并执行当前状态。
- `drive_step()` / `turn_step()`：用剩余距离或角度计算速度，到达后切换状态。
- `stop_before_exit()`：在关闭 ROS 通信之前连续发送零速度，处理 `Ctrl+C`。
- `include/course_bot_control/motion_math.hpp`：与 ROS 无关的角度、四元数和位移计算，可单独测试。
- `path_follower.cpp`：订阅 `/planned_path`、`/course_bot/world_pose` 和 `/odom`；每 0.1 秒选择下一路径点并控制 `/cmd_vel`。原始 `/odom` 用于检测车体倾斜。
- `include/course_bot_control/path_following_math.hpp`：先转向、再低速前进的计算，附有独立 C++ 测试。

## 构建和运行

项目源码在 VMware 共享目录，构建输出放在 Ubuntu 本机，避开共享目录的符号链接限制。启动 Gazebo 的入口是 XML 配置文件；控制节点仍由 C++ 编译产生。

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_bringup simulation.launch.xml
```

另开一个终端，重新 `source` 后，先低速测试直行，再测试旋转和矩形。不要同时运行两个控制器。

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run course_bot_control basic_motion --ros-args \
  -p mode:=straight -p target_distance:=0.3 -p linear_speed:=0.1

ros2 run course_bot_control basic_motion --ros-args -p mode:=rotate
ros2 run course_bot_control basic_motion --ros-args -p mode:=rectangle
```

单段距离最多 1 米，线速度最多 0.3 米/秒，角速度最多 0.6 弧度/秒。超过 1 秒没有 `/odom`、车身倾斜超过 20°、横向偏移过大或动作超时都会触发停车。矩形轨迹是里程计练习，**还不具备避障能力**。

## A* 路径跟踪：第一次仿真验收

请重新启动 Gazebo，让小车停在 `(-4,-3)` 生成点。**不要同时运行 `basic_motion` 或其他向 `/cmd_vel` 发速度的程序。** 按以下顺序使用三个终端：

终端 A：构建并启动仿真。

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_bringup simulation.launch.xml
```

终端 B：等小车生成后启动规划节点，确认出现 `A* 成功`。运行它时不要移动小车，否则首次 `/odom` 校准会错位。

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run course_bot_planner astar_planner --ros-args \
  -p use_sim_time:=true -p goal_x:=4.0 -p goal_y:=3.0
```

终端 C：确认地图和路径显示正常后，再启动跟踪节点。首次演示建议保持低速：

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run course_bot_control path_follower --ros-args \
  -p use_sim_time:=true -p max_linear:=0.08 -p max_angular:=0.25
```

节点先等待规划路径与实时地图位姿，随后依次转向、前进，到 `(4,3)` 附近停车。到达后三个终端都保持运行；在 RViz 使用 **2D Goal Pose** 点击另一处灰色自由区域，规划器会从小车实时位置产生新路径，跟踪器收到不同路径时先发零速度，再开始下一段。因此可以连续执行“初始点 → 第2点 → 第3点”，无需重启或手动设置新的起点。

如果规划器或跟踪器被 `Ctrl+C` 中断，先停止跟踪器以确保零速度，再停止规划器；Gazebo 可以继续运行。重新启动规划器时，它会从 `/gazebo/model_states` 获取 `course_bot` 的当前世界位姿并重新规划，随后再启动跟踪器即可继续。只有首次安装本功能、修改世界文件或重新构建后，才需要重启一次 Gazebo 让新的世界插件生效。

路径跟踪器同时订阅 `/scan`。默认情况下，前方约 ±30° 内小于 `0.40 m` 时禁止继续前进，但允许原地转向执行绕行路线；全周任意方向小于 `0.25 m` 时停止所有运动。`laser_stop_distance`、`laser_emergency_distance` 和 `laser_front_half_angle` 均可通过 `--ros-args -p` 调整。超过 1 秒没有收到 `/scan` 时会进入安全故障并停车。

动态障碍验收方法：保持三个节点运行，在 Gazebo 的 Insert 面板临时加入一个箱体并把它放在绿色路径前方约 1～2 米处，**不要保存世界**。规划器应打印“动态障碍地图已更新”和“重新运行 A*”，RViz 路径应绕开新箱体；箱体移走约 4 秒后临时占据区会自动清除。

请全程观察 Gazebo；如果车朝障碍物行驶，立即点击 Gazebo **暂停**，再按终端 C 的 `Ctrl+C`。停止后检查 `/map`、路径和 `map → odom` 是否对齐，勿直接重复运行。

安全限制：路径或位姿中断、车体倾斜超过 20°、偏离路径、运行超时或 `Ctrl+C` 均会发布零速度。人工换目标导致路径改变时，跟踪器会先停车再接受新路径；规划失败时空路径会取消当前运动。它只沿已知静态障碍的 A* 路线走，**不会用激光扫描实时避障**；下一步才考虑动态感知。

如果控制程序异常退出后车继续运动，先暂停 Gazebo，再运行：

```bash
ros2 run course_bot_control stop_robot
```

它会连续发送约 2 秒零速度。如果又开始运动，检查是否有其他节点向 `/cmd_vel` 发非零命令。

## C++ 测试

与 ROS 无关的数学部分可以单独编译测试；完整节点仍需在 Ubuntu 22.04 + ROS 2 Humble 中编译与运行验收。

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course/src/course_bot_control
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude test/test_motion_math.cpp -o /tmp/course_bot_motion_math_test
/tmp/course_bot_motion_math_test

g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  test/test_path_following_math.cpp -o /tmp/course_bot_path_math_test
/tmp/course_bot_path_math_test
```
