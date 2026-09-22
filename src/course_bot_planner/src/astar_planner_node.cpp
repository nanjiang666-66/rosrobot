// 第 6 阶段第一步：把已验证的 C++ A* 地图与路径发布到 ROS 2。
// 本节点只规划和显示路线，不向 /cmd_vel 发送任何速度。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "course_bot_planner/course_world_map.hpp"
#include "course_bot_planner/odom_anchor.hpp"
#include "course_bot_planner/sdf_world_loader.hpp"

namespace {

// Gazebo 发出的四元数转平面航向；先归一化以免噪声影响角度。
std::optional<double> yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-9) return std::nullopt;
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void set_yaw(geometry_msgs::msg::Quaternion &q, double yaw) {
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
}

std::string point_text(double x, double y) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(2);
  stream << '(' << x << ", " << y << ')';
  return stream.str();
}

}  // namespace

class AstarPlannerNode final : public rclcpp::Node {
public:
  AstarPlannerNode() : Node("course_bot_astar_planner") {
    const double start_x = declare_parameter<double>(
        "start_x", course_bot_planner::kDefaultStartWorld.x);
    const double start_y = declare_parameter<double>(
        "start_y", course_bot_planner::kDefaultStartWorld.y);
    goal_.x = declare_parameter<double>("goal_x", course_bot_planner::kDefaultGoalWorld.x);
    goal_.y = declare_parameter<double>("goal_y", course_bot_planner::kDefaultGoalWorld.y);
    plan_initial_goal_ = declare_parameter<bool>("plan_initial_goal", true);
    have_goal_ = plan_initial_goal_;
    use_gazebo_model_states_ = declare_parameter<bool>("use_gazebo_model_states", true);
    robot_model_name_ = declare_parameter<std::string>("robot_model_name", "course_bot");
    enable_dynamic_obstacles_ = declare_parameter<bool>("enable_dynamic_obstacles", true);
    dynamic_scan_max_range_ = declare_parameter<double>("dynamic_scan_max_range", 2.5);
    dynamic_obstacle_ttl_ = declare_parameter<double>("dynamic_obstacle_ttl", 4.0);
    dynamic_obstacle_radius_ = declare_parameter<double>("dynamic_obstacle_radius", 0.35);
    dynamic_confirm_scans_ = declare_parameter<int>("dynamic_confirm_scans", 3);
    laser_offset_x_ = declare_parameter<double>("laser_offset_x", 0.10);
    initial_world_pose_ = {start_x, start_y, 0.0};  // 当前 Gazebo 生成朝向是 0。

    const std::string default_world_file =
        ament_index_cpp::get_package_share_directory("course_bot_gazebo") +
        "/worlds/course_obstacles.world";
    const std::string map_source = declare_parameter<std::string>("map_source", "sdf");
    const std::string world_file =
        declare_parameter<std::string>("world_file", default_world_file);
    if (map_source == "sdf") {
      const auto loaded = course_bot_planner::load_static_obstacles_from_sdf(world_file);
      map_ = std::make_unique<course_bot_planner::CourseWorldMap>(loaded.obstacles);
      RCLCPP_INFO(
          get_logger(),
          "已从 SDF 自动建立地图：%zu 个静态障碍碰撞体，跳过 %zu 个水平地面 Plane、%zu 个动态模型。文件：%s",
          loaded.obstacles.size(), loaded.skipped_horizontal_planes,
          loaded.skipped_dynamic_models, world_file.c_str());
    } else if (map_source == "builtin") {
      map_ = std::make_unique<course_bot_planner::CourseWorldMap>();
      RCLCPP_WARN(get_logger(), "正在使用内置调试地图；它不会随 SDF 世界自动更新。");
    } else {
      throw std::invalid_argument("map_source 只能是 'sdf' 或 'builtin'");
    }
    if (!(std::isfinite(dynamic_scan_max_range_) && dynamic_scan_max_range_ >= 0.5 &&
          dynamic_scan_max_range_ <= 5.0)) {
      throw std::invalid_argument("dynamic_scan_max_range 必须在 0.5～5.0 米之间");
    }
    if (!(std::isfinite(dynamic_obstacle_ttl_) && dynamic_obstacle_ttl_ >= 1.0 &&
          dynamic_obstacle_ttl_ <= 15.0)) {
      throw std::invalid_argument("dynamic_obstacle_ttl 必须在 1.0～15.0 秒之间");
    }
    if (!(std::isfinite(dynamic_obstacle_radius_) && dynamic_obstacle_radius_ >= 0.0 &&
          dynamic_obstacle_radius_ <= 0.6)) {
      throw std::invalid_argument("dynamic_obstacle_radius 必须在 0～0.6 米之间");
    }
    if (dynamic_confirm_scans_ < 1 || dynamic_confirm_scans_ > 10) {
      throw std::invalid_argument("dynamic_confirm_scans 必须在 1～10 之间");
    }

    // 保存一份不会变化的 SDF 静态地图；运行时地图在它上面叠加临时障碍。
    static_grid_.emplace(map_->grid);
    const auto cell_count = static_cast<std::size_t>(map_->grid.width()) * map_->grid.height();
    dynamic_candidate_hits_.assign(cell_count, 0);
    dynamic_last_seen_.assign(cell_count, Clock::time_point{});
    dynamic_mask_.assign(cell_count, 0);

    const auto goal_cell = map_->world_to_cell(goal_);
    // 正常仿真直接读取 Gazebo 当前位姿；只有显式关闭真值定位时才检查备用起点参数。
    if (!use_gazebo_model_states_) {
      const auto start_cell = map_->world_to_cell({start_x, start_y});
      if (!start_cell) {
        throw std::invalid_argument(
            "备用起点 " + point_text(start_x, start_y) +
            " 超出地图范围；x、y 必须位于 [-5.00, 5.00) 米内");
      }
      if (!map_->grid.traversable(*start_cell)) {
        throw std::invalid_argument(
            "备用起点 " + point_text(start_x, start_y) +
            " 位于障碍物、安全膨胀区或地图边界保护区内");
      }
    }
    if (!goal_cell) {
      throw std::invalid_argument(
          "目标点 " + point_text(goal_.x, goal_.y) +
          " 超出地图范围；x、y 必须位于 [-5.00, 5.00) 米内");
    }
    if (!map_->grid.traversable(*goal_cell)) {
      throw std::invalid_argument(
          "目标点 " + point_text(goal_.x, goal_.y) +
          " 位于障碍物、安全膨胀区或地图边界保护区内");
    }

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planned_path", latched_qos);
    world_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("/course_bot/world_pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&AstarPlannerNode::on_odom, this, std::placeholders::_1));
    if (use_gazebo_model_states_) {
      model_states_subscription_ = create_subscription<gazebo_msgs::msg::ModelStates>(
          "/gazebo/model_states", rclcpp::QoS(1).best_effort(),
          std::bind(&AstarPlannerNode::on_model_states, this, std::placeholders::_1));
    }
    if (enable_dynamic_obstacles_) {
      scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", rclcpp::SensorDataQoS(),
          std::bind(&AstarPlannerNode::on_scan, this, std::placeholders::_1));
    }
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&AstarPlannerNode::on_goal, this, std::placeholders::_1));
    // 5 Hz 检查动态地图，让路径受阻后最多约 0.2 秒便开始重新规划。
    republish_timer_ = create_wall_timer(std::chrono::milliseconds(200),
                                         std::bind(&AstarPlannerNode::on_timer, this));

    build_map_message();
    if (use_gazebo_model_states_ && plan_initial_goal_) {
      RCLCPP_INFO(get_logger(),
                  "等待 /odom 与 /gazebo/model_states 中的模型 '%s'；将从小车当前世界位姿规划。"
                  "初始目标 (%.2f, %.2f)，之后可用 RViz 的 2D Goal Pose 连续换目标。",
                  robot_model_name_.c_str(), goal_.x, goal_.y);
    } else if (use_gazebo_model_states_) {
      RCLCPP_INFO(get_logger(),
                  "等待 /odom、/gazebo/model_states 和 RViz 的 2D Goal Pose；不会自动驶向默认终点。");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Gazebo 真值定位已关闭：首次 /odom 将按备用起点 (%.2f, %.2f) 校准。",
                  start_x, start_y);
    }
    if (enable_dynamic_obstacles_) {
      RCLCPP_INFO(get_logger(),
                  "动态障碍检测已启用：/scan 最大 %.1f 米，连续 %d 帧确认，%.1f 秒未观测后移除。",
                  dynamic_scan_max_range_, dynamic_confirm_scans_, dynamic_obstacle_ttl_);
    }
  }

private:
  using Clock = std::chrono::steady_clock;

  void build_map_message() {
    map_message_.header.frame_id = "map";
    map_message_.info.resolution = course_bot_planner::CourseWorldMap::kResolution;
    map_message_.info.width = course_bot_planner::CourseWorldMap::kWidth;
    map_message_.info.height = course_bot_planner::CourseWorldMap::kHeight;
    map_message_.info.origin.position.x = course_bot_planner::CourseWorldMap::kOriginX;
    map_message_.info.origin.position.y = course_bot_planner::CourseWorldMap::kOriginY;
    map_message_.info.origin.orientation.w = 1.0;
    // OccupancyGrid 按行存储：索引 = y * width + x；0 自由、100 占据。
    map_message_.data.resize(static_cast<std::size_t>(map_->grid.width()) * map_->grid.height());
    for (int y = 0; y < map_->grid.height(); ++y) {
      for (int x = 0; x < map_->grid.width(); ++x) {
        const auto index = static_cast<std::size_t>(y) * map_->grid.width() + x;
        const course_bot_planner::Cell cell{x, y};
        if (static_grid_ && !static_grid_->traversable(cell)) {
          map_message_.data[index] = 100;  // SDF 静态障碍和边界。
        } else if (!map_->grid.traversable(cell)) {
          map_message_.data[index] = 80;   // 激光检测到的临时障碍。
        } else {
          map_message_.data[index] = 0;
        }
      }
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message) {
    const auto &position = message->pose.pose.position;
    const auto yaw = yaw_from_quaternion(message->pose.pose.orientation);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !yaw) {
      RCLCPP_WARN(get_logger(), "收到无效 /odom 位姿，忽略本条消息。");
      return;
    }
    const course_bot_planner::Pose2D odom_pose{position.x, position.y, *yaw};
    latest_odom_pose_ = odom_pose;
    latest_odom_frame_ = message->header.frame_id;
    const bool initialized_now = try_initialize_anchor();
    if (!anchor_) return;  // 真值模式下，可能还没收到 /gazebo/model_states。
    latest_world_pose_ = anchor_->odom_pose_in_map(odom_pose);
    publish_world_pose(*latest_world_pose_, message->header.stamp);
    if (initialized_now && have_goal_) plan_from(*latest_world_pose_);
  }

  void on_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr message) {
    const auto iterator = std::find(
        message->name.begin(), message->name.end(), robot_model_name_);
    if (iterator == message->name.end()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "/gazebo/model_states 中找不到模型 '%s'；请检查 spawn_entity.py 的 -entity 名称。",
          robot_model_name_.c_str());
      return;
    }
    const auto index = static_cast<std::size_t>(std::distance(message->name.begin(), iterator));
    if (index >= message->pose.size()) {
      RCLCPP_WARN(get_logger(), "/gazebo/model_states 的名称与位姿数量不一致，忽略本条消息。");
      return;
    }
    const auto &pose = message->pose[index];
    const auto yaw = yaw_from_quaternion(pose.orientation);
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !yaw) {
      RCLCPP_WARN(get_logger(), "Gazebo 返回的模型位姿无效，忽略本条消息。");
      return;
    }
    latest_gazebo_world_pose_ = {pose.position.x, pose.position.y, *yaw};
    const bool initialized_now = try_initialize_anchor();
    if (!anchor_) return;  // 可能还没收到 /odom。

    // Gazebo 给出的是真实世界位姿，后续换目标时优先把它作为当前起点。
    latest_world_pose_ = latest_gazebo_world_pose_;
    publish_world_pose(
        *latest_world_pose_, static_cast<builtin_interfaces::msg::Time>(now()));
    if (initialized_now && have_goal_) plan_from(*latest_world_pose_);
  }

  bool try_initialize_anchor() {
    if (anchor_ || !latest_odom_pose_) return false;
    if (use_gazebo_model_states_ && !latest_gazebo_world_pose_) return false;

    const auto world_pose = use_gazebo_model_states_
                                ? *latest_gazebo_world_pose_
                                : initial_world_pose_;
    anchor_.emplace(*latest_odom_pose_, world_pose);
    publish_map_to_odom(latest_odom_frame_);
    RCLCPP_INFO(get_logger(), "定位初始化完成：当前世界位姿 (%.2f, %.2f)，偏航 %.2f 弧度。",
                world_pose.x, world_pose.y, world_pose.yaw);
    return true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message) {
    if (!enable_dynamic_obstacles_ || !latest_world_pose_ || !static_grid_) return;
    if (!std::isfinite(message->angle_min) || !std::isfinite(message->angle_increment) ||
        message->angle_increment <= 0.0 || !std::isfinite(message->range_min) ||
        !std::isfinite(message->range_max) || message->range_max <= message->range_min) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "/scan 参数无效，暂不更新动态障碍。");
      return;
    }

    std::vector<std::uint8_t> observed(dynamic_mask_.size(), 0);
    const auto robot = *latest_world_pose_;
    // URDF 中 laser_link 位于 base_link 前方 0.10 米；这里把激光原点变换到 map。
    const double laser_x = robot.x + laser_offset_x_ * std::cos(robot.yaw);
    const double laser_y = robot.y + laser_offset_x_ * std::sin(robot.yaw);
    const double usable_max = std::min(dynamic_scan_max_range_,
                                       static_cast<double>(message->range_max));

    for (std::size_t i = 0; i < message->ranges.size(); ++i) {
      const double range = message->ranges[i];
      // 等于最大量程通常表示“没有命中”，不能把它误画成一圈障碍物。
      if (!std::isfinite(range) || range < message->range_min ||
          range >= usable_max || range >= message->range_max) {
        continue;
      }
      const double beam_angle = robot.yaw + message->angle_min +
                                static_cast<double>(i) * message->angle_increment;
      const course_bot_planner::WorldPoint endpoint{
          laser_x + range * std::cos(beam_angle),
          laser_y + range * std::sin(beam_angle)};
      const auto cell = map_->world_to_cell(endpoint);
      if (!cell || !static_grid_->traversable(*cell)) {
        continue;  // 已在 SDF 静态占据区内，不属于“新出现”的障碍物。
      }
      observed[cell_index(*cell)] = 1;
    }

    const auto now = Clock::now();
    for (std::size_t i = 0; i < observed.size(); ++i) {
      if (observed[i]) {
        dynamic_candidate_hits_[i] =
            std::min(dynamic_candidate_hits_[i] + 1, dynamic_confirm_scans_);
        if (dynamic_candidate_hits_[i] >= dynamic_confirm_scans_) {
          dynamic_last_seen_[i] = now;
        }
      } else {
        dynamic_candidate_hits_[i] = std::max(0, dynamic_candidate_hits_[i] - 1);
      }
    }
    rebuild_dynamic_grid(now);
  }

  std::size_t cell_index(course_bot_planner::Cell cell) const {
    return static_cast<std::size_t>(cell.y) * map_->grid.width() + cell.x;
  }

  course_bot_planner::Cell cell_from_index(std::size_t index) const {
    const auto width = static_cast<std::size_t>(map_->grid.width());
    return {static_cast<int>(index % width), static_cast<int>(index / width)};
  }

  bool rebuild_dynamic_grid(Clock::time_point now) {
    if (!enable_dynamic_obstacles_ || !static_grid_) return false;
    auto updated = *static_grid_;
    std::vector<std::uint8_t> new_mask(dynamic_mask_.size(), 0);
    std::size_t active_observations = 0;

    // 激光只看到物体表面，因此在机器人半径和安全余量之外，再加一段未知物体厚度。
    const double half_cell_diagonal =
        0.5 * std::sqrt(2.0) * course_bot_planner::CourseWorldMap::kResolution;
    const double inflation = course_bot_planner::CourseWorldMap::kRobotRadius +
                             course_bot_planner::CourseWorldMap::kSafetyMargin +
                             dynamic_obstacle_radius_ + half_cell_diagonal;
    const int cell_radius = static_cast<int>(
        std::ceil(inflation / course_bot_planner::CourseWorldMap::kResolution));

    for (std::size_t i = 0; i < dynamic_last_seen_.size(); ++i) {
      if (dynamic_last_seen_[i] == Clock::time_point{} ||
          std::chrono::duration<double>(now - dynamic_last_seen_[i]).count() >
              dynamic_obstacle_ttl_) {
        continue;
      }
      ++active_observations;
      const auto source = cell_from_index(i);
      const auto source_center = map_->cell_center(source);
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          const course_bot_planner::Cell cell{source.x + dx, source.y + dy};
          if (!updated.in_bounds(cell) || !static_grid_->traversable(cell)) continue;
          const auto center = map_->cell_center(cell);
          if (std::hypot(center.x - source_center.x, center.y - source_center.y) <=
              inflation) {
            updated.set_blocked(cell);
            new_mask[cell_index(cell)] = 1;
          }
        }
      }
    }

    // 激光点膨胀范围可能覆盖机器人当前格；只清除该格，保证 A* 仍有合法起点。
    if (latest_world_pose_) {
      const auto robot_cell = map_->world_to_cell(
          {latest_world_pose_->x, latest_world_pose_->y});
      if (robot_cell && static_grid_->traversable(*robot_cell)) {
        updated.set_blocked(*robot_cell, false);
        new_mask[cell_index(*robot_cell)] = 0;
      }
    }

    if (new_mask == dynamic_mask_) return false;
    dynamic_mask_ = std::move(new_mask);
    map_->grid = std::move(updated);
    build_map_message();
    dynamic_replan_pending_ = have_plan_result_ && path_intersects_blocked_cell();
    RCLCPP_INFO(get_logger(),
                "动态障碍地图已更新：%zu 个确认观测点；%s。",
                active_observations,
                dynamic_replan_pending_ ? "当前路径受影响，准备重新规划"
                                        : "当前路径未受影响");
    return true;
  }

  bool path_intersects_blocked_cell() const {
    if (path_message_.poses.empty()) return true;
    for (const auto &pose : path_message_.poses) {
      const auto cell = map_->world_to_cell(
          {pose.pose.position.x, pose.pose.position.y});
      if (!cell || !map_->grid.traversable(*cell)) return true;
    }
    return false;
  }

  void on_timer() {
    rebuild_dynamic_grid(Clock::now());  // 即使雷达停止发布，也能按 TTL 清除旧障碍。
    if (dynamic_replan_pending_ && latest_world_pose_) {
      dynamic_replan_pending_ = false;
      RCLCPP_WARN(get_logger(), "临时障碍占据当前路径，正从机器人实时位置重新运行 A*。");
      plan_from(*latest_world_pose_);
      return;
    }
    publish_cached();
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    if (!message->header.frame_id.empty() && message->header.frame_id != "map") {
      RCLCPP_ERROR(get_logger(),
                   "拒绝新目标：坐标系是 '%s'，当前仅接受 map 坐标系。",
                   message->header.frame_id.c_str());
      return;
    }
    const course_bot_planner::WorldPoint candidate{
        message->pose.position.x, message->pose.position.y};
    if (!std::isfinite(candidate.x) || !std::isfinite(candidate.y)) {
      RCLCPP_ERROR(get_logger(), "拒绝新目标：坐标不是有效数值。");
      return;
    }
    const auto cell = map_->world_to_cell(candidate);
    if (!cell) {
      RCLCPP_ERROR(get_logger(), "拒绝新目标 %s：超出地图范围 [-5.00, 5.00) 米。",
                   point_text(candidate.x, candidate.y).c_str());
      return;
    }
    if (!map_->grid.traversable(*cell)) {
      RCLCPP_ERROR(get_logger(),
                   "拒绝新目标 %s：位于障碍物、安全膨胀区或边界保护区。",
                   point_text(candidate.x, candidate.y).c_str());
      return;
    }

    goal_ = candidate;
    have_goal_ = true;
    if (!latest_world_pose_) {
      RCLCPP_INFO(get_logger(), "已接收新目标 %s；等待首次 /odom 后规划。",
                  point_text(goal_.x, goal_.y).c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "收到新目标 %s；从机器人当前位置重新运行 A*。",
                point_text(goal_.x, goal_.y).c_str());
    plan_from(*latest_world_pose_);
  }

  void publish_map_to_odom(std::string odom_frame) {
    if (odom_frame.empty()) odom_frame = "odom";
    if (!odom_frame.empty() && odom_frame.front() == '/') odom_frame.erase(0, 1);
    if (odom_frame == "map" || odom_frame.empty()) {
      throw std::runtime_error("/odom must use a distinct odom frame");
    }
    const auto transform = anchor_->map_to_odom_transform();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = odom_frame;
    tf.transform.translation.x = transform.x;
    tf.transform.translation.y = transform.y;
    set_yaw(tf.transform.rotation, transform.yaw);
    tf_broadcaster_->sendTransform(tf);
    RCLCPP_INFO(get_logger(), "已校准 map -> %s：平移 (%.2f, %.2f) 米，偏航 %.2f 弧度。",
                odom_frame.c_str(), transform.x, transform.y, transform.yaw);
  }

  void plan_from(course_bot_planner::Pose2D start_world) {
    dynamic_replan_pending_ = false;
    const auto start_cell = map_->world_to_cell({start_world.x, start_world.y});
    const auto goal_cell = map_->world_to_cell(goal_);
    nav_msgs::msg::Path new_path;
    new_path.header.frame_id = "map";
    if (!start_cell) {
      RCLCPP_ERROR(get_logger(), "当前起点 %s 超出地图范围；不发布运动路线。",
                   point_text(start_world.x, start_world.y).c_str());
      path_message_ = new_path;
      have_plan_result_ = true;
      publish_cached();
      return;
    }
    if (!map_->grid.traversable(*start_cell)) {
      RCLCPP_ERROR(get_logger(),
                   "当前起点 %s 位于障碍物、安全膨胀区或边界保护区；不发布运动路线。",
                   point_text(start_world.x, start_world.y).c_str());
      path_message_ = new_path;
      have_plan_result_ = true;
      publish_cached();
      return;
    }
    if (!goal_cell) {
      RCLCPP_ERROR(get_logger(), "目标点 %s 超出地图范围；不发布运动路线。",
                   point_text(goal_.x, goal_.y).c_str());
      path_message_ = new_path;
      have_plan_result_ = true;
      publish_cached();
      return;
    }
    if (!map_->grid.traversable(*goal_cell)) {
      RCLCPP_ERROR(get_logger(),
                   "目标点 %s 位于障碍物、安全膨胀区或边界保护区；不发布运动路线。",
                   point_text(goal_.x, goal_.y).c_str());
      path_message_ = new_path;
      have_plan_result_ = true;
      publish_cached();
      return;
    }
    const auto result = course_bot_planner::astar(
        map_->grid, *start_cell, *goal_cell, course_bot_planner::Connectivity::Eight);
    if (result.status != course_bot_planner::SearchStatus::Found) {
      RCLCPP_ERROR(get_logger(), "A* 规划失败：%s；/planned_path 保持为空。",
                   course_bot_planner::status_name(result.status));
      path_message_ = new_path;
      have_plan_result_ = true;
      publish_cached();
      return;
    }

    const auto add_pose = [&new_path](course_bot_planner::WorldPoint point) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.w = 1.0;  // 路径跟踪器以后根据相邻点计算目标朝向。
      new_path.poses.push_back(pose);
    };
    // 首尾使用准确的米制位置；中间的格子转为其中心坐标。
    add_pose({start_world.x, start_world.y});
    for (std::size_t i = 1; i < result.path.size(); ++i) {
      add_pose(map_->cell_center(result.path[i]));
    }
    add_pose(goal_);
    path_message_ = std::move(new_path);
    have_plan_result_ = true;
    publish_cached();
    RCLCPP_INFO(get_logger(), "A* 成功：%zu 个路径点，搜索 %zu 格，代价 %.2f。",
                path_message_.poses.size(), result.expanded_nodes, result.total_cost);
  }

  void publish_world_pose(course_bot_planner::Pose2D pose, const builtin_interfaces::msg::Time &stamp) {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = "map";
    message.header.stamp = stamp;
    message.pose.position.x = pose.x;
    message.pose.position.y = pose.y;
    set_yaw(message.pose.orientation, pose.yaw);
    world_pose_publisher_->publish(message);
  }

  void publish_cached() {
    if (!anchor_) return;  // 首条 /odom 到来后才完成坐标校准。
    const auto stamp = now();
    map_message_.header.stamp = stamp;
    map_message_.info.map_load_time = stamp;
    map_publisher_->publish(map_message_);
    if (have_plan_result_) {
      path_message_.header.stamp = stamp;
      for (auto &pose : path_message_.poses) pose.header.stamp = stamp;
      path_publisher_->publish(path_message_);
    }
  }

  std::unique_ptr<course_bot_planner::CourseWorldMap> map_;
  std::optional<course_bot_planner::Grid> static_grid_;
  course_bot_planner::Pose2D initial_world_pose_{};
  course_bot_planner::WorldPoint goal_{};
  bool use_gazebo_model_states_{true};
  bool plan_initial_goal_{true};
  bool have_goal_{true};
  std::string robot_model_name_{"course_bot"};
  bool enable_dynamic_obstacles_{true};
  double dynamic_scan_max_range_{2.5};
  double dynamic_obstacle_ttl_{4.0};
  double dynamic_obstacle_radius_{0.35};
  int dynamic_confirm_scans_{3};
  double laser_offset_x_{0.10};
  std::vector<int> dynamic_candidate_hits_;
  std::vector<Clock::time_point> dynamic_last_seen_;
  std::vector<std::uint8_t> dynamic_mask_;
  bool dynamic_replan_pending_{false};
  std::optional<course_bot_planner::OdomAnchor> anchor_;
  std::optional<course_bot_planner::Pose2D> latest_odom_pose_;
  std::optional<course_bot_planner::Pose2D> latest_gazebo_world_pose_;
  std::string latest_odom_frame_{"odom"};
  std::optional<course_bot_planner::Pose2D> latest_world_pose_;
  nav_msgs::msg::OccupancyGrid map_message_;
  nav_msgs::msg::Path path_message_;
  bool have_plan_result_{false};
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_pose_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<AstarPlannerNode>());
  } catch (const std::exception &error) {
    std::cerr << "A* 规划节点异常：" << error.what() << '\n';
    status = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
