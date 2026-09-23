# course_bot_slam

本包包含两种启动方式：

- `slam_mapping.launch.xml`：课程建图总入口，一条命令启动 Gazebo、机器人、SLAM 和 RViz。
- `mapping.launch.xml`：底层建图入口，只启动 SLAM 和 RViz，供 Gazebo 已运行时复用。
- `navigation.launch.xml`：稳定课程导航，启动 Gazebo、SDF 基础地图、自编 C++ A*、
  激光临时障碍检测、自编 C++ 路径跟踪器和 RViz。

## 重要限制

`slam_mapping.launch.xml` 和 `navigation.launch.xml` 是两个独立实验，不能同时运行：SLAM 与
SDF 规划器都会发布 `/map` 和 `map → odom`。课程自主导航默认使用稳定的 SDF 模式；
SLAM 建图暂时单独学习和保存，不接管导航。

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
