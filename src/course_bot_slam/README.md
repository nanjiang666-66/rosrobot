# course_bot_slam

本包包含以下启动方式：

- `slam_mapping.launch.xml`：课程建图总入口，一条命令启动 Gazebo、机器人、SLAM 和 RViz。
- `challenge_mapping.launch.xml`：独立交错隔墙世界的建图入口。
- `challenge_navigation.launch.xml`：加载独立挑战地图，演示明显绕行的自编 A* 导航。
- `mapping.launch.xml`：底层建图入口，只启动 SLAM 和 RViz，供 Gazebo 已运行时复用。
- `slam_navigation.launch.xml`：加载已保存位姿图，运行 SLAM 定位、自编 A* 和路径跟踪。
- `navigation.launch.xml`：稳定课程导航，启动 Gazebo、SDF 基础地图、自编 C++ A*、
  激光临时障碍检测、自编 C++ 路径跟踪器和 RViz。

## 重要限制

普通与挑战模式的建图、SLAM 导航，以及 `navigation.launch.xml` 都应分别运行，不能
同时启动。建图与 SLAM 导航会各自启动 `slam_toolbox`；SDF 导航还会独立发布 `/map` 和
`map → odom`。请先关闭上一种模式，再启动下一种。

## 一键启动建图

终端 A：

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_slam slam_mapping.launch.xml
```

如果 Gazebo 已在另一个终端运行，改用：

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_slam slam_mapping.launch.xml start_simulation:=false
```

启动后先不要移动机器人，等待 Gazebo、机器人模型、LaserScan 和灰色未知地图在 RViz 中出现。

## 检查建图数据链

终端 B：

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 topic hz /scan
```

看到雷达频率后按 `Ctrl+C`，再依次检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

两段 TF 都持续输出后按 `Ctrl+C`。最后确认 SLAM 已发布地图和定位变换：

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

只有 `/scan`、`odom → base_footprint`、`base_footprint → laser_link`、`/map` 和
`map → odom` 都正常时，才开始移动机器人。

## 手动遥控扫描地图

推荐在终端 B 使用键盘遥控：

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

如果提示找不到包，可安装 Ubuntu 22.04 / ROS 2 Humble 对应工具：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

保持低速，先在起点附近前后移动和转向，确认地图与机器人位置稳定，再沿可行区域逐步扫描。
不要撞击障碍，不要同时启动 `basic_motion`、路径跟踪器或其他 `/cmd_vel` 发布节点。

没有键盘遥控工具时，也可在确认无障碍的区域运行 `basic_motion`：

先用起点附近的小矩形验证地图会随运动扩展：

```bash
ros2 run course_bot_control basic_motion --ros-args \
  -p mode:=rectangle -p long_side:=0.8 -p short_side:=0.5 \
  -p linear_speed:=0.08 -p angular_speed:=0.20
```

这个矩形只用于初步验收，不能覆盖整个世界；完整建图仍需要安全地探索其他可通行区域。

## 保存地图和 SLAM 位姿图

扫描完成后，先停止机器人，但保持 Gazebo 和 `slam_toolbox` 运行。以下文件名前缀不要添加
`.pgm`、`.yaml` 或 `.posegraph` 扩展名。

终端 B 保存可视化占据地图：

```bash
ros2 run nav2_map_server map_saver_cli -f \
  /mnt/hgfs/ubuntu-workspace/robot_nav_course/src/course_bot_slam/maps/course_map \
  --ros-args -p map_subscribe_transient_local:=true
```

再保存可供 SLAM Toolbox 定位模式加载的序列化位姿图：

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/mnt/hgfs/ubuntu-workspace/robot_nav_course/src/course_bot_slam/maps/course_map'}"
```

成功后应得到 `course_map.pgm`、`course_map.yaml`、`course_map.posegraph` 和
`course_map.data`。

立即核对四个文件：

```bash
ls -lh /mnt/hgfs/ubuntu-workspace/robot_nav_course/src/course_bot_slam/maps/course_map.*
```

`PGM/YAML` 是可视化占据地图；`posegraph/data` 保存激光节点、约束和位姿图。下一阶段使用
SLAM Toolbox 定位时四个文件都保留，尤其不能只保留 `PGM/YAML`。地图文件生成后再次运行
`bash scripts/stage2_build.sh`，才能把它们复制到安装空间供后续启动文件读取。

## 第一批验收标准

1. 一条 `slam_mapping.launch.xml` 命令能启动仿真、SLAM 和 RViz。
2. RViz 中未知区域为灰色、扫描后的自由区变白、障碍边界变黑。
3. 机器人运动时，地图不会跟随机器人整体漂移或不断重影。
4. `/map` 和 `map → odom` 持续存在，且规划器没有发布第二份 `/map`。
5. `maps/` 中生成并核对四个 `course_map` 文件。

本阶段只完成手动建图与保存，不启动 A*、路径跟踪器或自主探索。

## SLAM 固定地图定位与 A* 导航

前提是 `maps/` 已经包含同名前缀的四个 `course_map` 文件。先关闭建图模式、键盘遥控、
旧 SDF 规划器和其他 `/cmd_vel` 发布节点，然后重新构建：

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_slam slam_navigation.launch.xml
```

该命令会启动：

1. Gazebo 和机器人；
2. SLAM Toolbox 定位模式，并加载 `course_map.posegraph/data`；
3. 只消费 `/map` 和 TF 的自编 C++ `slam_astar_planner`；
4. C++ `path_follower`；
5. RViz。

保存地图中的障碍物坐标与 Gazebo 世界坐标基本对齐。机器人由
`course_bot_gazebo/launch/gazebo.launch.xml` 固定生成在 `(-4.0, -3.0)`，所以定位初值设为
`map_start_pose: [-4.0, -3.0, 0.0]`。这个值只告诉定位节点从地图哪里开始匹配激光，
不会移动 Gazebo 机器人。等待地图、机器人模型和 `map → odom → base_footprint` 稳定，
检查 RViz 红色激光点是否与黑色障碍边界重合，再设置导航目标。

如果只重启 RViz/定位节点而 Gazebo 继续运行，小车会留在上次停止位置；如果改变机器人
生成点，默认初值也不再适用。这两种情况都应先在 RViz 顶部选择 **2D Pose Estimate**，
在地图中小车当前实际位置点击并拖出朝向，等待激光与地图匹配。无法确认实际位置时，
关闭旧仿真并重新启动 Gazebo，让机器人回到固定生成点，再重新启动定位导航。

运行中修正位姿时，SLAM 导航会先取消旧路径并停车，等待新的定位数据稳定约 1.2 秒，
然后从最新位置为尚未完成的目标重新运行 A*；通常不需要重启 Gazebo。如果跟踪器暂时
收不到位姿、里程计或激光，也会先停车，等数据恢复后请求新路径。地图更新但未挡住
剩余路线时不会重新规划。到达终点后旧目标会被清除，不会因地图更新而再次启动。
这些恢复仅针对本次运行中的定位修正；目前**不会保存上次停车位姿**，重启 Gazebo
仍从固定生成点启动。手动给出错误的 `2D Pose Estimate` 可能让激光与地图不匹配，
应先确认红色扫描点与地图障碍边界对齐，再发送目标。

定位参数里的 `map_file_name` 是当前虚拟机安装空间的绝对路径。若更换 Linux 用户名或构建
目录，需同步修改 `config/mapper_params_localization.yaml`，重新构建后再启动。

定位正常后，在 RViz 顶部选择 **2D Goal Pose**，只能在已经扫描出的白色自由区点击目标。
规划器会把机器人当前 `map` 坐标作为起点，运行自编 A*，发布 `/planned_path`，路径跟踪器再
发布 `/cmd_vel`。目标箭头的方向暂不参与最终姿态控制，只使用目标 `x/y`。

正式验收模式固定为：

- 不读取 Gazebo SDF 障碍坐标；
- 不订阅 `/gazebo/model_states`；
- 未知灰色区域不可通行；
- 关闭规划器的临时动态障碍层；
- 雷达仍保留近距离紧急停车保护；
- 不进行自主探索。

因此，建图后才临时放入 Gazebo 的新障碍不会成为本次 A* 的长期地图来源。小车靠近它时会
停车，但不会把它作为主要验收内容自动绕行；正式演示应使用建图时已经扫描并保存的障碍。

常用检查命令：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /map --once
ros2 topic echo /goal_pose --once
ros2 topic echo /planned_path --once
ros2 topic info /map -v
```

`/map` 应由 `slam_toolbox` 发布，不能同时出现旧 SDF 规划器。规划失败时，C++ 节点会分别提示
目标超出地图、位于未知区域、原始障碍物、安全膨胀区，或 A* 没有可行路径。

可以在启动命令末尾调整路径跟踪速度：

```bash
ros2 launch course_bot_slam slam_navigation.launch.xml \
  max_linear:=0.10 max_angular:=0.30
```

如果 Gazebo 已经运行，可追加 `start_simulation:=false`。不要让建图模式和定位导航模式同时
运行，因为它们都会启动 `slam_toolbox` 并发布 `map → odom`。

## 稳定导航：基础地图 + 临时障碍

先关闭此前单独运行的 Gazebo、SLAM、规划器和跟踪器，然后构建并启动：

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_slam navigation.launch.xml
```

可在启动时直接指定最大直线速度和最大转向速度，无需修改 C++：

```bash
ros2 launch course_bot_slam navigation.launch.xml \
  max_linear:=0.10 \
  max_angular:=0.30
```

`max_linear` 单位为 m/s，允许范围为 0.04～0.15；`max_angular` 单位为 rad/s，
允许范围为 0.10～0.50。学习和调试时建议不要超过 `0.10/0.30`。

速度参数在启动时由 `path_follower` 读取；当前代码没有运行时参数回调，所以
`ros2 param set` 即使显示设置成功，也不会立即改变车速。需要先安全停车，
再用新的 `max_linear` / `max_angular` 参数重启对应的导航入口。

启动完成后：

1. 等待固定 10 m × 10 m 地图、机器人和雷达出现。
2. 在 RViz 顶部选择 **2D Goal Pose**。
3. 在地图自由区域点击并拖出箭头。
4. 绿色 `/planned_path` 出现后，机器人会自动跟踪路线。
5. 可连续点击其他终点；A* 每次都从机器人实时位置重新规划。

启动时，规划器从 `course_obstacles.world` 自动读取所有 `<static>true</static>` 的箱体、
圆柱和球体，生成基础地图。运行过程中临时加入、没有保存到 SDF 的物体由 `/scan` 在
5 米内识别，连续确认后加入临时障碍层；当前路径被挡住时以 0.2 秒周期重新规划，消失
4 秒后从临时层移除。近距离安全层仍会立即停车。

目标位于地图外、固定障碍、临时障碍或安全膨胀区时会被拒绝。RViz 箭头方向暂不参与
最终姿态控制，只使用终点的 `x/y`。启动后默认原地等待，不会自动前往旧默认终点。

若 Gazebo 已经在另一个终端运行，只启动其余组件：

```bash
ros2 launch course_bot_slam navigation.launch.xml start_simulation:=false
```

不要同时运行 `basic_motion`、键盘遥控或其他 `/cmd_vel` 发布节点。

常用检查命令：

```bash
ros2 topic echo /goal_pose --once
ros2 topic echo /planned_path --once
ros2 topic echo /course_bot/world_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

`/map` 是包含安全膨胀区和临时障碍的规划地图。SDF 中后来保存的新静态模型需重启规划
节点后读取；仿真运行中临时加入的模型不需要重启，由雷达动态识别。

## 独立的交错隔墙 A* 演示

这套演示不覆盖原来的 `course_obstacles.world` 或 `course_map.*`。新世界中的红墙从南边界
延伸，只能从上方绕过；蓝墙从北边界延伸，只能从下方绕过。固定起点为 `(-4,-3)`，推荐
终点为 `(4,-3)`。因此路径需要先向北绕红墙，再向南绕蓝墙，而不是直接横穿墙壁。
这不是让 A* 故意走远；它仍在实际可通行区域内搜索最短路径。
挑战入口的安全余量默认 `0.20 m`，比普通导航的 `0.10 m` 更宽，转墙角时更稳妥。

先关闭现有的 Gazebo、SLAM 和导航程序，在终端 A 构建并进入挑战建图模式：

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_slam challenge_mapping.launch.xml
```

终端 B 用键盘遥控逐步扫描：先从红墙北端绕入两墙之间，再从蓝墙南端绕到东侧，尽量扫描
整条通道和边界，最后可回到起点附近帮助闭环。建图阶段没有自主防撞，请低速操作：

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

确认 RViz 中起点、两个绕行口、终点附近都已变成白色已知区域，黑色墙体清晰后，停止
小车但保持建图节点运行。在终端 C 保存**独立**地图与位姿图：

```bash
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f \
  /mnt/hgfs/ubuntu-workspace/robot_nav_course/src/course_bot_slam/maps/challenge_map \
  --ros-args -p map_subscribe_transient_local:=true
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/mnt/hgfs/ubuntu-workspace/robot_nav_course/src/course_bot_slam/maps/challenge_map'}"
```

检查 `challenge_map.pgm`、`.yaml`、`.posegraph` 和 `.data` 四个文件都已生成。然后关闭
建图、遥控和 Gazebo，**再次构建**，把新地图安装到 ROS 工作空间，最后启动挑战导航：

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
ros2 launch course_bot_slam challenge_navigation.launch.xml \
  max_linear:=0.10 max_angular:=0.30
```

等激光点与黑色墙体对齐后，在 RViz 用 **2D Goal Pose** 点击右侧 `(4,-3)` 附近的白色
区域。另一种方法是终端发布目标：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 4.0, y: -3.0, z: 0.0}, orientation: {w: 1.0}}}"
```

挑战定位参数使用 `config/mapper_params_localization_challenge.yaml`，只读取
`challenge_map.posegraph/data`。如果更换 Ubuntu 用户名或构建目录，需要修改该文件中的
`map_file_name` 绝对路径并重新构建。**不能把原 `course_map.*` 直接用于新世界**：地图和
实际墙体不匹配会造成定位错误及碰撞风险。
