// 第 7B 阶段：在 slam_toolbox 生成的 /map 上运行自编 C++ A*。
//
// 与旧 astar_planner 的区别：
// 1. 本节点订阅 /map，不读取 Gazebo SDF；
// 2. 本节点从 TF(map -> base_footprint) 获取实时位姿；
// 3. 本节点绝不发布 /map 或 map -> odom，避免和 slam_toolbox 冲突；
// 4. RViz 的 2D Goal Pose 发布 /goal_pose 后才开始规划。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "course_bot_planner/course_world_map.hpp"
#include "course_bot_planner/odom_anchor.hpp"  // 复用只含 x、y、yaw 的 Pose2D 数据结构。

namespace {

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

bool same_grid(const course_bot_planner::Grid &a,
               const course_bot_planner::Grid &b) {
  if (a.width() != b.width() || a.height() != b.height()) return false;
  for (int y = 0; y < a.height(); ++y) {
    for (int x = 0; x < a.width(); ++x) {
      const course_bot_planner::Cell cell{x, y};
      if (a.traversable(cell) != b.traversable(cell)) return false;
      // 未知格逐渐变成已知自由格时，通行性不变但代价会变化，也需要重新规划。
      if (a.traversable(cell) && std::abs(a.cost(cell) - b.cost(cell)) > 1e-9) return false;
    }
  }
  return true;
}

}  // namespace

class SlamAstarPlannerNode final : public rclcpp::Node {
public:
  SlamAstarPlannerNode() : Node("course_bot_slam_astar_planner") {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
    unknown_is_blocked_ = declare_parameter<bool>("unknown_is_blocked", true);
    unknown_cost_ = declare_parameter<double>("unknown_cost", 4.0);
    robot_radius_ = declare_parameter<double>(
        "robot_radius", course_bot_planner::CourseWorldMap::kRobotRadius);
    safety_margin_ = declare_parameter<double>(
        "safety_margin", course_bot_planner::CourseWorldMap::kSafetyMargin);
    enable_dynamic_obstacles_ = declare_parameter<bool>("enable_dynamic_obstacles", true);
    dynamic_scan_max_range_ = declare_parameter<double>("dynamic_scan_max_range", 2.5);
    dynamic_obstacle_ttl_ = declare_parameter<double>("dynamic_obstacle_ttl", 4.0);
    dynamic_obstacle_radius_ = declare_parameter<double>("dynamic_obstacle_radius", 0.35);
    dynamic_confirm_scans_ = declare_parameter<int>("dynamic_confirm_scans", 3);
    laser_offset_x_ = declare_parameter<double>("laser_offset_x", 0.10);
    validate_parameters();

    // 规划路径采用 transient_local，新启动的跟踪器也能收到最近一次路径。
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planned_path", latched_qos);
    planning_map_publisher_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>("/course_bot/planning_map", latched_qos);
    world_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("/course_bot/world_pose", 10);

    // slam_toolbox 的 /map 是可靠、持久化话题；使用相同 QoS 才能稳定匹配。
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", latched_qos,
        std::bind(&SlamAstarPlannerNode::on_map, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&SlamAstarPlannerNode::on_goal, this, std::placeholders::_1));
    if (enable_dynamic_obstacles_) {
      scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", rclcpp::SensorDataQoS(),
          std::bind(&SlamAstarPlannerNode::on_scan, this, std::placeholders::_1));
    }

    // TF 来自 slam_toolbox：map -> odom；机器人自身发布 odom -> base_footprint。
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // 5 Hz 获取位姿、清理过期动态障碍并重发路径。
    timer_ = create_wall_timer(std::chrono::milliseconds(200),
                               std::bind(&SlamAstarPlannerNode::on_timer, this));
    RCLCPP_INFO(get_logger(),
                "SLAM A* 已启动：等待 /map 和 TF map -> %s；请在 RViz 使用 2D Goal Pose。",
                base_frame_.c_str());
    RCLCPP_INFO(get_logger(),
                "未知区域按%s处理；障碍膨胀半径 %.2f 米。",
                unknown_is_blocked_ ? "不可通行" : "高代价探索区",
                robot_radius_ + safety_margin_);
    if (!unknown_is_blocked_) {
      RCLCPP_WARN(get_logger(),
                  "未知区探索已启用：A* 代价 %.1f，优先走已知区；激光负责途中检测和重规划。",
                  unknown_cost_);
    }
  }

private:
  using Clock = std::chrono::steady_clock;

  void validate_parameters() const {
    if (base_frame_.empty()) throw std::invalid_argument("base_frame 不能为空");
    if (occupied_threshold_ < 1 || occupied_threshold_ > 100) {
      throw std::invalid_argument("occupied_threshold 必须在 1～100 之间");
    }
    if (!(std::isfinite(unknown_cost_) && unknown_cost_ >= 1.0 && unknown_cost_ <= 20.0)) {
      throw std::invalid_argument("unknown_cost 必须在 1～20 之间");
    }
    if (!(std::isfinite(robot_radius_) && robot_radius_ >= 0.10 && robot_radius_ <= 0.60)) {
      throw std::invalid_argument("robot_radius 必须在 0.10～0.60 米之间");
    }
    if (!(std::isfinite(safety_margin_) && safety_margin_ >= 0.0 && safety_margin_ <= 0.50)) {
      throw std::invalid_argument("safety_margin 必须在 0～0.50 米之间");
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
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
    if (message->header.frame_id != "map") {
      RCLCPP_ERROR(get_logger(), "拒绝地图：/map 的 frame_id 必须是 map，实际是 '%s'。",
                   message->header.frame_id.c_str());
      return;
    }
    const auto width = static_cast<int>(message->info.width);
    const auto height = static_cast<int>(message->info.height);
    const double resolution = message->info.resolution;
    const auto expected = static_cast<std::size_t>(message->info.width) * message->info.height;
    const auto origin_yaw = yaw_from_quaternion(message->info.origin.orientation);
    if (width <= 0 || height <= 0 || expected != message->data.size() ||
        !(std::isfinite(resolution) && resolution > 0.0) || !origin_yaw) {
      RCLCPP_ERROR(get_logger(), "拒绝地图：尺寸、分辨率、原点姿态或 data 长度无效。");
      return;
    }
    // 当前坐标转换按轴对齐地图实现；slam_toolbox 正常发布的地图原点偏航为 0。
    if (std::abs(*origin_yaw) > 1e-3) {
      RCLCPP_ERROR(get_logger(), "拒绝地图：地图原点偏航 %.4f，不是轴对齐地图。", *origin_yaw);
      return;
    }

    const double origin_x = message->info.origin.position.x;
    const double origin_y = message->info.origin.position.y;
    auto incoming = std::make_unique<course_bot_planner::CourseWorldMap>(
        width, height, resolution, origin_x, origin_y);

    // inflation_sources 只包含实体障碍，以及未知区边缘。
    // 不膨胀未知区深处，能避免每秒对大面积未知格做重复计算。
    std::vector<course_bot_planner::Cell> inflation_sources;
    const auto index = [width](int x, int y) {
      return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
             static_cast<std::size_t>(x);
    };
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto value = message->data[index(x, y)];
        const bool occupied = value >= occupied_threshold_;
        const bool unknown = value < 0;
        const bool blocked = occupied || (unknown_is_blocked_ && unknown);
        if (blocked) incoming->grid.set_blocked({x, y});
        // 未知区允许探索时仍提高进入代价，避免 A* 为了几格距离盲目穿越大片未知区域。
        if (unknown && !unknown_is_blocked_) {
          incoming->grid.set_cost({x, y}, unknown_cost_);
        }
        if (occupied) inflation_sources.push_back({x, y});
      }
    }
    if (unknown_is_blocked_) {
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          if (message->data[index(x, y)] >= 0) continue;
          bool borders_known_free = false;
          for (int dy = -1; dy <= 1 && !borders_known_free; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
              const int nx = x + dx;
              const int ny = y + dy;
              if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
              const auto neighbor = message->data[index(nx, ny)];
              if (neighbor >= 0 && neighbor < occupied_threshold_) {
                borders_known_free = true;
                break;
              }
            }
          }
          if (borders_known_free) inflation_sources.push_back({x, y});
        }
      }
    }

    // 把机器人外接圆和安全余量换算成栅格，对原始障碍做圆形膨胀。
    const double inflation = robot_radius_ + safety_margin_ +
                             0.5 * std::sqrt(2.0) * resolution;
    const int cell_radius = static_cast<int>(std::ceil(inflation / resolution));
    for (const auto source : inflation_sources) {
      const auto source_center = incoming->cell_center(source);
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          const course_bot_planner::Cell cell{source.x + dx, source.y + dy};
          if (!incoming->grid.in_bounds(cell)) continue;
          const auto center = incoming->cell_center(cell);
          if (std::hypot(center.x - source_center.x, center.y - source_center.y) <= inflation) {
            incoming->grid.set_blocked(cell);
          }
        }
      }
    }

    const bool same_geometry = map_ && map_->grid.width() == width &&
        map_->grid.height() == height &&
        std::abs(map_->resolution() - resolution) < 1e-9 &&
        std::abs(map_->origin_x() - origin_x) < 1e-9 &&
        std::abs(map_->origin_y() - origin_y) < 1e-9;
    const bool changed = !static_grid_ || !same_geometry ||
                         !same_grid(*static_grid_, incoming->grid);

    map_ = std::move(incoming);
    static_grid_.emplace(map_->grid);
    map_load_time_ = message->info.map_load_time;
    if (!same_geometry) reset_dynamic_storage();
    rebuild_dynamic_grid(Clock::now());
    build_planning_map_message();
    publish_planning_map();

    if (!received_first_map_) {
      received_first_map_ = true;
      RCLCPP_INFO(get_logger(), "已接收 SLAM 地图：%d × %d 格，分辨率 %.3f 米。",
                  width, height, resolution);
    }
    if (changed && have_goal_ && latest_world_pose_) {
      RCLCPP_INFO(get_logger(), "SLAM 地图有变化，从机器人当前位置重新规划。");
      plan_from(*latest_world_pose_);
    }
  }

  void reset_dynamic_storage() {
    if (!map_) return;
    const auto count = static_cast<std::size_t>(map_->grid.width()) * map_->grid.height();
    dynamic_candidate_hits_.assign(count, 0);
    dynamic_last_seen_.assign(count, Clock::time_point{});
    dynamic_mask_.assign(count, 0);
  }

  void update_pose_from_tf() {
    try {
      const auto transform = tf_buffer_->lookupTransform(
          "map", base_frame_, tf2::TimePointZero);
      const auto yaw = yaw_from_quaternion(transform.transform.rotation);
      if (!yaw || !std::isfinite(transform.transform.translation.x) ||
          !std::isfinite(transform.transform.translation.y)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "TF 返回无效机器人位姿。");
        return;
      }
      const bool first_pose = !latest_world_pose_;
      latest_world_pose_ = {transform.transform.translation.x,
                            transform.transform.translation.y, *yaw};
      publish_world_pose(*latest_world_pose_);
      if (first_pose && map_ && have_goal_) plan_from(*latest_world_pose_);
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "等待 TF map -> %s：%s", base_frame_.c_str(), error.what());
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    if (!message->header.frame_id.empty() && message->header.frame_id != "map") {
      RCLCPP_ERROR(get_logger(), "拒绝新目标：坐标系是 '%s'，只接受 map。",
                   message->header.frame_id.c_str());
      return;
    }
    const course_bot_planner::WorldPoint candidate{
        message->pose.position.x, message->pose.position.y};
    if (!std::isfinite(candidate.x) || !std::isfinite(candidate.y)) {
      RCLCPP_ERROR(get_logger(), "拒绝新目标：坐标不是有效数值。");
      return;
    }
    if (!map_) {
      goal_ = candidate;
      have_goal_ = true;
      RCLCPP_INFO(get_logger(), "已暂存目标 %s；等待首张 /map。",
                  point_text(goal_.x, goal_.y).c_str());
      return;
    }
    const auto cell = map_->world_to_cell(candidate);
    if (!cell) {
      RCLCPP_ERROR(get_logger(), "拒绝新目标 %s：超出当前 SLAM 地图范围。",
                   point_text(candidate.x, candidate.y).c_str());
      return;
    }
    if (!map_->grid.traversable(*cell)) {
      RCLCPP_ERROR(get_logger(),
                   "拒绝新目标 %s：位于障碍物或安全膨胀区。",
                   point_text(candidate.x, candidate.y).c_str());
      return;
    }
    goal_ = candidate;
    have_goal_ = true;
    if (!latest_world_pose_) {
      RCLCPP_INFO(get_logger(), "已接收目标 %s；等待 TF 定位。",
                  point_text(goal_.x, goal_.y).c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "收到 RViz 目标 %s，从实时位置运行 A*。",
                point_text(goal_.x, goal_.y).c_str());
    plan_from(*latest_world_pose_);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message) {
    if (!enable_dynamic_obstacles_ || !map_ || !static_grid_ || !latest_world_pose_) return;
    if (!std::isfinite(message->angle_min) || !std::isfinite(message->angle_increment) ||
        message->angle_increment <= 0.0 || !std::isfinite(message->range_min) ||
        !std::isfinite(message->range_max) || message->range_max <= message->range_min) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "/scan 参数无效，暂不更新动态障碍。");
      return;
    }
    if (dynamic_mask_.size() !=
        static_cast<std::size_t>(map_->grid.width()) * map_->grid.height()) {
      reset_dynamic_storage();
    }

    std::vector<std::uint8_t> observed(dynamic_mask_.size(), 0);
    const auto robot = *latest_world_pose_;
    const double laser_x = robot.x + laser_offset_x_ * std::cos(robot.yaw);
    const double laser_y = robot.y + laser_offset_x_ * std::sin(robot.yaw);
    const double usable_max = std::min(dynamic_scan_max_range_,
                                       static_cast<double>(message->range_max));
    for (std::size_t i = 0; i < message->ranges.size(); ++i) {
      const double range = message->ranges[i];
      if (!std::isfinite(range) || range < message->range_min ||
          range >= usable_max || range >= message->range_max) continue;
      const double beam_angle = robot.yaw + message->angle_min +
                                static_cast<double>(i) * message->angle_increment;
      const course_bot_planner::WorldPoint endpoint{
          laser_x + range * std::cos(beam_angle),
          laser_y + range * std::sin(beam_angle)};
      const auto cell = map_->world_to_cell(endpoint);
      // SLAM 静态地图已占据的点无需再加入“临时障碍”层。
      if (!cell || !static_grid_->traversable(*cell)) continue;
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
    if (!map_ || !static_grid_ || dynamic_mask_.empty()) return false;
    auto updated = *static_grid_;
    std::vector<std::uint8_t> new_mask(dynamic_mask_.size(), 0);
    const double half_cell_diagonal = 0.5 * std::sqrt(2.0) * map_->resolution();
    const double inflation = robot_radius_ + safety_margin_ +
                             dynamic_obstacle_radius_ + half_cell_diagonal;
    const int cell_radius = static_cast<int>(std::ceil(inflation / map_->resolution()));

    for (std::size_t i = 0; i < dynamic_last_seen_.size(); ++i) {
      if (dynamic_last_seen_[i] == Clock::time_point{} ||
          std::chrono::duration<double>(now - dynamic_last_seen_[i]).count() >
              dynamic_obstacle_ttl_) continue;
      const auto source = cell_from_index(i);
      const auto source_center = map_->cell_center(source);
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          const course_bot_planner::Cell cell{source.x + dx, source.y + dy};
          if (!updated.in_bounds(cell) || !static_grid_->traversable(cell)) continue;
          const auto center = map_->cell_center(cell);
          if (std::hypot(center.x - source_center.x, center.y - source_center.y) <= inflation) {
            updated.set_blocked(cell);
            new_mask[cell_index(cell)] = 1;
          }
        }
      }
    }
    // 激光端点偶尔会落到车体所在格，允许机器人从当前格开始搜索。
    if (latest_world_pose_) {
      const auto robot_cell = map_->world_to_cell(
          {latest_world_pose_->x, latest_world_pose_->y});
      if (robot_cell && static_grid_->traversable(*robot_cell)) {
        updated.set_blocked(*robot_cell, false);
        new_mask[cell_index(*robot_cell)] = 0;
      }
    }
    if (new_mask == dynamic_mask_ && same_grid(updated, map_->grid)) return false;
    dynamic_mask_ = std::move(new_mask);
    map_->grid = std::move(updated);
    build_planning_map_message();
    dynamic_replan_pending_ = have_plan_result_ && path_intersects_blocked_cell();
    return true;
  }

  bool path_intersects_blocked_cell() const {
    if (!map_ || path_message_.poses.empty()) return true;
    for (const auto &pose : path_message_.poses) {
      const auto cell = map_->world_to_cell(
          {pose.pose.position.x, pose.pose.position.y});
      if (!cell || !map_->grid.traversable(*cell)) return true;
    }
    return false;
  }

  void plan_from(course_bot_planner::Pose2D start_world) {
    if (!map_ || !have_goal_) return;
    dynamic_replan_pending_ = false;
    const auto start_cell = map_->world_to_cell({start_world.x, start_world.y});
    const auto goal_cell = map_->world_to_cell(goal_);
    nav_msgs::msg::Path new_path;
    new_path.header.frame_id = "map";
    if (!start_cell) {
      publish_plan_failure(new_path, "当前起点 " + point_text(start_world.x, start_world.y) +
                                     " 超出 SLAM 地图范围");
      return;
    }
    if (!map_->grid.traversable(*start_cell)) {
      publish_plan_failure(new_path, "当前起点 " + point_text(start_world.x, start_world.y) +
                                     " 位于障碍物或膨胀区");
      return;
    }
    if (!goal_cell) {
      publish_plan_failure(new_path, "目标点 " + point_text(goal_.x, goal_.y) +
                                     " 超出当前 SLAM 地图范围");
      return;
    }
    if (!map_->grid.traversable(*goal_cell)) {
      publish_plan_failure(new_path, "目标点 " + point_text(goal_.x, goal_.y) +
                                     " 位于障碍物或膨胀区");
      return;
    }

    const auto result = course_bot_planner::astar(
        map_->grid, *start_cell, *goal_cell, course_bot_planner::Connectivity::Eight);
    if (result.status != course_bot_planner::SearchStatus::Found) {
      publish_plan_failure(new_path,
                           std::string("A* 规划失败：") +
                           course_bot_planner::status_name(result.status));
      return;
    }
    const auto add_pose = [&new_path](course_bot_planner::WorldPoint point) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.w = 1.0;
      new_path.poses.push_back(pose);
    };
    add_pose({start_world.x, start_world.y});
    for (std::size_t i = 1; i < result.path.size(); ++i) {
      add_pose(map_->cell_center(result.path[i]));
    }
    add_pose(goal_);
    path_message_ = std::move(new_path);
    have_plan_result_ = true;
    publish_cached();
    RCLCPP_INFO(get_logger(), "SLAM A* 成功：%zu 个路径点，搜索 %zu 格，代价 %.2f。",
                path_message_.poses.size(), result.expanded_nodes, result.total_cost);
  }

  void publish_plan_failure(nav_msgs::msg::Path &empty_path, const std::string &reason) {
    path_message_ = std::move(empty_path);
    have_plan_result_ = true;
    publish_cached();
    RCLCPP_ERROR(get_logger(), "%s；机器人保持停车。", reason.c_str());
  }

  void build_planning_map_message() {
    if (!map_ || !static_grid_) return;
    planning_map_message_.header.frame_id = "map";
    planning_map_message_.info.resolution = map_->resolution();
    planning_map_message_.info.width = static_cast<std::uint32_t>(map_->grid.width());
    planning_map_message_.info.height = static_cast<std::uint32_t>(map_->grid.height());
    planning_map_message_.info.origin.position.x = map_->origin_x();
    planning_map_message_.info.origin.position.y = map_->origin_y();
    planning_map_message_.info.origin.orientation.w = 1.0;
    planning_map_message_.info.map_load_time = map_load_time_;
    planning_map_message_.data.resize(
        static_cast<std::size_t>(map_->grid.width()) * map_->grid.height());
    for (int y = 0; y < map_->grid.height(); ++y) {
      for (int x = 0; x < map_->grid.width(); ++x) {
        const course_bot_planner::Cell cell{x, y};
        const auto i = cell_index(cell);
        if (!static_grid_->traversable(cell)) {
          planning_map_message_.data[i] = 100;
        } else if (!map_->grid.traversable(cell)) {
          planning_map_message_.data[i] = 80;
        } else if (static_grid_->cost(cell) > 1.0) {
          planning_map_message_.data[i] = -1;  // 允许通行但尚未由 SLAM 确认的未知格。
        } else {
          planning_map_message_.data[i] = 0;
        }
      }
    }
  }

  void publish_world_pose(course_bot_planner::Pose2D pose) {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = "map";
    message.header.stamp = now();
    message.pose.position.x = pose.x;
    message.pose.position.y = pose.y;
    set_yaw(message.pose.orientation, pose.yaw);
    world_pose_publisher_->publish(message);
  }

  void publish_planning_map() {
    if (!map_) return;
    planning_map_message_.header.stamp = now();
    planning_map_publisher_->publish(planning_map_message_);
  }

  void publish_cached() {
    publish_planning_map();
    if (!have_plan_result_) return;
    const auto stamp = now();
    path_message_.header.stamp = stamp;
    for (auto &pose : path_message_.poses) pose.header.stamp = stamp;
    path_publisher_->publish(path_message_);
  }

  void on_timer() {
    update_pose_from_tf();
    const bool dynamic_changed = rebuild_dynamic_grid(Clock::now());
    if (dynamic_changed) publish_planning_map();
    if (dynamic_replan_pending_ && latest_world_pose_ && have_goal_) {
      dynamic_replan_pending_ = false;
      RCLCPP_WARN(get_logger(), "临时障碍占据当前路径，重新运行 SLAM A*。");
      plan_from(*latest_world_pose_);
      return;
    }
    publish_cached();
  }

  std::string base_frame_{"base_footprint"};
  int occupied_threshold_{50};
  bool unknown_is_blocked_{true};
  double unknown_cost_{4.0};
  double robot_radius_{0.31};
  double safety_margin_{0.10};
  bool enable_dynamic_obstacles_{true};
  double dynamic_scan_max_range_{2.5};
  double dynamic_obstacle_ttl_{4.0};
  double dynamic_obstacle_radius_{0.35};
  int dynamic_confirm_scans_{3};
  double laser_offset_x_{0.10};

  std::unique_ptr<course_bot_planner::CourseWorldMap> map_;
  std::optional<course_bot_planner::Grid> static_grid_;
  std::optional<course_bot_planner::Pose2D> latest_world_pose_;
  course_bot_planner::WorldPoint goal_{};
  bool have_goal_{false};
  bool received_first_map_{false};
  bool have_plan_result_{false};
  bool dynamic_replan_pending_{false};
  std::vector<int> dynamic_candidate_hits_;
  std::vector<Clock::time_point> dynamic_last_seen_;
  std::vector<std::uint8_t> dynamic_mask_;
  builtin_interfaces::msg::Time map_load_time_;
  nav_msgs::msg::OccupancyGrid planning_map_message_;
  nav_msgs::msg::Path path_message_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_pose_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<SlamAstarPlannerNode>());
  } catch (const std::exception &error) {
    std::cerr << "SLAM A* 规划节点异常：" << error.what() << '\n';
    status = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
