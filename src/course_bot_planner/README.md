# C++ A*：独立算法、仿真地图与 ROS 2 规划节点

本包的 A* 核心**不调用 ROS API**；另有一个很薄的 C++ ROS 2 节点把静态地图和规划路径发布出来。整个包**不会向 `/cmd_vel` 发命令**，所以当前还不能自主行驶。地图有两张：教学用小地图，以及按现有 Gazebo 世界的三个静态障碍物建立的 10 m × 10 m 栅格地图。

## 读代码的顺序

1. `include/course_bot_planner/astar.hpp`：格子 `Cell`、地图 `Grid`、搜索结果的数据结构。
2. `src/astar.cpp`：A* 的优先队列、`g + h`、邻居检查和路径回溯。
3. `test/test_astar.cpp`：每个场景的预期结果，可以边读边运行。
4. `src/astar_demo.cpp`：把一次规划结果打印为字符地图。
5. `include/course_bot_planner/course_world_map.hpp` 和 `src/course_world_map.cpp`：仿真世界几何、障碍膨胀和坐标转换。
6. `src/world_map_demo.cpp`：在仿真世界对应的栅格中预览规划路线。
7. `include/course_bot_planner/odom_anchor.hpp`、`src/odom_anchor.cpp`：把首条 `/odom` 与已知 Gazebo 生成点对齐。
8. `src/astar_planner_node.cpp`：旧 SDF 模式，发布 `/map`、`/planned_path` 和真值定位。
9. `src/slam_astar_planner_node.cpp`：SLAM 模式，订阅 `/map` 与 TF，接受 RViz 终点。

每个空地的进入代价默认是 1。`set_blocked()` 标记不可通行的障碍；`set_cost()` 可以把一个格子设为高代价区域。搜索中 `g` 是已经走过的实际代价，`h` 是到终点的估计代价，优先队列按 `f = g + h` 取出候选格子。四方向使用曼哈顿距离；八方向使用八方向距离，并禁止从两个障碍物的墙角斜穿。

`SearchResult` 会明确区分成功、起点无效、终点无效和无路可达。成功时 `path` 按“起点到终点”的顺序排列；失败时 `path` 为空。

## 在现有 Ubuntu 工作空间运行

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run course_bot_planner astar_demo
```

演示程序也可以指定起终点的**栅格坐标**，例如：

```bash
ros2 run course_bot_planner astar_demo 1 1 10 6
```

打印结果中，`S` 是起点，`G` 是终点，`#` 是障碍，`F` 是高代价格，`*` 是路径。默认演示地图是独立例子，**尚未与 Gazebo 的 `course_obstacles.world` 对齐**。

## 仿真世界对应的静态地图

`CourseWorldMap` 覆盖世界坐标 `x,y ∈ [-5,5)` 米，每格 0.20 米，共 50 × 50 格。运行节点会从 `course_bot_gazebo/worlds/course_obstacles.world` 自动读取这些静态碰撞体：

| Gazebo 模型 | 中心位置（米） | 平面形状 |
| --- | --- | --- |
| `obstacle_box_1` | `(-1.0,-1.2)` | 1.0 × 1.0 米箱体，朝向 0 |
| `obstacle_box_2` | `(1.5,1.0)` | 1.4 × 0.7 米箱体，朝向 0.35 弧度 |
| `obstacle_cylinder` | `(3.0,-1.5)` | 半径 0.55 米圆柱 |
| `obstacle_sphere` | `(0.5,-2.7)` | 半径 0.45 米球体的地面投影 |
| `unit_box` | `(0.40,2.90)` | 1.0 × 1.0 米箱体 |

生成地图时并非仅把障碍物的原始轮廓标为 `#`，而是再向外留出机器人外接圆半径 0.31 米、安全余量 0.10 米、半格对角线约 0.14 米。地图边界也保留这一距离，因此打印图上的 `#` 范围会明显大于 Gazebo 里的实体。这样搜索得到的路径才不会让车身贴障碍行驶。

当前默认世界坐标起点是 Gazebo 生成位置 `(-4.0,-3.0)`；预选终点是空地 `(4.0,3.0)`。运行时的实际起点由 `/gazebo/model_states` 自动读取，两者都由 `world_to_cell()` 转成格子，路径点可由 `cell_center()` 转回米制坐标。Gazebo 世界中的静态箱体、圆柱和球体会从 SDF 自动载入；机器人外形或安全余量改变后仍需调整膨胀半径并重跑测试。

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run course_bot_planner world_map_demo
```

也可以用四个米制坐标测试另一组起终点，例如：

```bash
ros2 run course_bot_planner world_map_demo -4 -3 4 3
```

字符图从上到下是世界坐标的正 `y` 到负 `y`；`#` 表示膨胀后的障碍或规划边界，`*` 表示 A* 找出的格子路径。它仍只是**规划演示**，不会让 Gazebo 小车运动。

## 在 ROS 2 中发布地图和路径

先构建。在终端 A 启动仿真，等待机器人生成，**不要先启动运动控制节点**：

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_bringup simulation.launch.xml
```

新开终端 B 启动 C++ 规划节点；如需换终点，修改 `goal_x` 和 `goal_y` 的米制数值：

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run course_bot_planner astar_planner --ros-args \
  -p use_sim_time:=true -p goal_x:=4.0 -p goal_y:=3.0
```

如果终点超出 `[-5,5)` 米的地图范围，或落在障碍物、安全膨胀区、边界保护区中，节点会打印带具体坐标的中文错误并退出，不会发布可执行路径。

正常情况下终端 B 会先报告 `map -> odom` 校准结果，再报告 `A* 成功`。节点发布：

| 话题或 TF | 类型 | 作用 |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | 0 表示自由，100 表示膨胀障碍；地图坐标系是 `map` |
| `/planned_path` | `nav_msgs/Path` | 从当前位置到目标的米制路径点 |
| `/course_bot/world_pose` | `geometry_msgs/PoseStamped` | 用于检查小车在 `map` 坐标系中的位置 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 的新目标输入；每次收到后从实时位置重新规划 |
| `map → odom` | 静态 TF | 把现有机器人 TF 树接到地图坐标系 |

规划器还会订阅 `/scan`：雷达命中点如果落在 SDF 静态地图的自由区，就被视为运行时新增障碍。连续三帧确认后，该点按机器人半径、安全余量和未知物体厚度进行膨胀；若它占据当前路径，规划器以 0.2 秒周期检查并自动从实时位置再次运行 A*。RViz 的 `/map` 中，静态障碍值为 100，临时障碍值为 80。临时障碍超过 4 秒未再被观测会移除。

可调参数：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `enable_dynamic_obstacles` | `true` | 是否启用雷达临时障碍和自动重规划 |
| `dynamic_scan_max_range` | `2.5` | 参与临时障碍检测的最大雷达距离（米） |
| `dynamic_confirm_scans` | `3` | 同一栅格连续确认帧数，抑制单帧噪声 |
| `dynamic_obstacle_ttl` | `4.0` | 未再次观测后保留时间（秒） |
| `dynamic_obstacle_radius` | `0.35` | 对未知物体表面追加的保守厚度（米） |

终端 C 可检查是否发布成功：

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 topic list -t
ros2 topic echo /planned_path --once
ros2 topic echo /course_bot/world_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

在 RViz 中将 **Fixed Frame** 设为 `map`，添加 **Map** 显示并选 `/map`，添加 **Path** 显示并选 `/planned_path`，还可添加 **TF**。地图和路径每秒重发一次，便于在 RViz 启动较晚时仍能显示。

首次路径运行完成后，直接在 RViz 顶部选择 **2D Goal Pose**，在灰色自由区域点击（拖动方向目前会被忽略）。规划器会从机器人实时位置重新运行 A*，发布下一条路径。目标在地图外或黑色膨胀障碍中时会被拒绝，原路线不变。

**重启恢复：**默认世界加载 `gazebo_ros_state` 插件并发布 `/gazebo/model_states`。规划节点每次启动都会读取名为 `course_bot` 的当前世界位姿，再与当前 `/odom` 建立 `map → odom` 校准，所以小车已经移动后可只重启规划器和跟踪器，不必重启 Gazebo。`start_x/start_y` 只在显式设置 `-p use_gazebo_model_states:=false` 时作为备用起点；`robot_model_name` 可用于覆盖模型名。不要同时运行其他发布 `map → odom` 的定位节点。

旧 `astar_planner` 支持连续目标、激光临时障碍和自动重规划，但使用 Gazebo 真值位姿。
第 7B 新增的 `slam_astar_planner` 改为读取 SLAM `/map` 和 TF 定位，并且不会发布
`/map` 或 `map → odom`。完整启动方法见 `course_bot_slam/README.md`。

## 不依赖 ROS 的单独测试

可在 Ubuntu 终端直接编译测试，输出放到本机 `/tmp`，避免 VMware 共享目录的符号链接限制：

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course/src/course_bot_planner
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  src/astar.cpp test/test_astar.cpp -o /tmp/course_bot_astar_test
/tmp/course_bot_astar_test

g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  src/astar.cpp src/course_world_map.cpp test/test_course_world_map.cpp \
  -o /tmp/course_bot_world_map_test
/tmp/course_bot_world_map_test

g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  src/odom_anchor.cpp test/test_odom_anchor.cpp \
  -o /tmp/course_bot_odom_anchor_test
/tmp/course_bot_odom_anchor_test
```

测试覆盖：空地图最短路径、绕墙、窄通道、高代价绕行、无路可达、起终点无效、起终点相同、八方向移动、禁止斜穿墙角，以及 Gazebo 世界障碍位置、坐标转换、默认路线和 `odom` 对齐。

## 两种规划节点不要混用

- SDF 真值导航：`ros2 run course_bot_planner astar_planner ...`
- SLAM 导航：`ros2 run course_bot_planner slam_astar_planner`

运行 SLAM Toolbox 时只能选第二种。速度控制仍由独立的 C++ `path_follower` 节点完成。
