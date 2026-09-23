// 沿 A* 发布的 /planned_path 行驶，并用激光雷达做最后一道近距离保护。
// 临时障碍的地图更新和重规划由规划节点负责；本节点只保证来不及时立即停车。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "course_bot_control/motion_math.hpp"
#include "course_bot_control/laser_safety.hpp"
#include "course_bot_control/path_following_math.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
}  // namespace

class PathFollower final : public rclcpp::Node {
public:
  PathFollower() : Node("course_bot_path_follower") {
    max_linear_ = declare_parameter<double>("max_linear", 0.10);
    max_angular_ = declare_parameter<double>("max_angular", 0.30);
    // SLAM 入口开启后，路径偏差和短暂数据中断会请求新路径；旧 SDF 入口保持原行为。
    recoverable_path_errors_ = declare_parameter<bool>("recoverable_path_errors", false);
    laser_stop_distance_ = declare_parameter<double>("laser_stop_distance", 0.40);
    laser_emergency_distance_ = declare_parameter<double>("laser_emergency_distance", 0.25);
    laser_front_half_angle_ = declare_parameter<double>("laser_front_half_angle", 0.52);
    if (!(std::isfinite(max_linear_) && max_linear_ >= 0.04 && max_linear_ <= 0.15)) {
      throw std::invalid_argument("max_linear 必须在 0.04～0.15 米/秒之间");
    }
    if (!(std::isfinite(max_angular_) && max_angular_ >= 0.10 && max_angular_ <= 0.50)) {
      throw std::invalid_argument("max_angular 必须在 0.10～0.50 弧度/秒之间");
    }
    if (!(std::isfinite(laser_stop_distance_) && laser_stop_distance_ >= 0.30 &&
          laser_stop_distance_ <= 1.00)) {
      throw std::invalid_argument("laser_stop_distance 必须在 0.30～1.00 米之间");
    }
    if (!(std::isfinite(laser_emergency_distance_) &&
          laser_emergency_distance_ >= 0.15 &&
          laser_emergency_distance_ < laser_stop_distance_)) {
      throw std::invalid_argument(
          "laser_emergency_distance 必须在 0.15 米以上并小于 laser_stop_distance");
    }
    if (!(std::isfinite(laser_front_half_angle_) && laser_front_half_angle_ >= 0.10 &&
          laser_front_half_angle_ <= 1.20)) {
      throw std::invalid_argument("laser_front_half_angle 必须在 0.10～1.20 弧度之间");
    }

    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    replan_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/course_bot/replan_request", 10);
    goal_reached_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/course_bot/goal_reached", 10);
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10, std::bind(&PathFollower::on_path, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/course_bot/world_pose", rclcpp::SensorDataQoS(),
        std::bind(&PathFollower::on_pose, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&PathFollower::on_odom, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&PathFollower::on_scan, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&PathFollower::control_step, this));
    RCLCPP_INFO(get_logger(),
                "等待 /planned_path、/course_bot/world_pose、/odom 和 /scan；未收到前保持停车。");
  }

  void stop_before_exit() {
    // 关闭 ROS 通信前连续发零速度，避免 Gazebo 保留上一个非零 /cmd_vel。
    RCLCPP_INFO(get_logger(), "收到退出信号，发送零速度停车。");
    for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
      publish_stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

private:
  enum class State { Waiting, Following, Done, Fault };
  using Clock = std::chrono::steady_clock;
  static constexpr double kWaypointTolerance = 0.13;
  static constexpr double kGoalTolerance = 0.15;

  static double point_distance(course_bot_control::PlanarPoint a,
                               course_bot_control::PlanarPoint b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr message) {
    if (state_ == State::Fault) return;
    if (message->header.frame_id != "map") {
      stop_with_fault("路径必须在 map 坐标系");
      return;
    }
    if (message->poses.empty()) {
      const bool had_path = !path_.empty();
      const bool completed = state_ == State::Done;
      publish_stop();
      path_.clear();
      target_index_ = 0;
      state_ = State::Waiting;
      last_path_time_ = Clock::now();
      if (had_path && !completed) {
        RCLCPP_WARN(get_logger(), "规划器取消了当前路径，机器人停车等待新路径。");
      }
      return;
    }
    // 等待传感器恢复时，即使规划器重发了缓存路径，也不能提前启动车轮。
    if (waiting_for_sensors_) return;
    if (message->poses.size() < 2) {
      stop_with_fault("非空路径至少需要起点和终点两个位置");
      return;
    }
    std::vector<course_bot_control::PlanarPoint> incoming;
    incoming.reserve(message->poses.size());
    for (const auto &pose : message->poses) {
      const auto &position = pose.pose.position;
      if ((!pose.header.frame_id.empty() && pose.header.frame_id != "map") ||
          !std::isfinite(position.x) || !std::isfinite(position.y)) {
        stop_with_fault("路径点坐标或坐标系无效");
        return;
      }
      incoming.push_back({position.x, position.y});
    }

    bool same = incoming.size() == path_.size();
    if (same) {
      for (std::size_t i = 0; i < path_.size(); ++i) {
        if (point_distance(incoming[i], path_[i]) > 1e-6) {
          same = false;
          break;
        }
      }
    }
    last_path_time_ = Clock::now();
    if (same) return;  // 规划器会定时重发同一路径，只刷新存活时间。

    const bool replacing = !path_.empty();
    publish_stop();  // 换目标时先停车，再从新路径起点开始跟踪。
    path_ = std::move(incoming);
    target_index_ = 0;
    state_ = State::Waiting;
    if (replacing) {
      RCLCPP_INFO(get_logger(), "收到新的 %zu 点路径：已停车并准备重新跟踪。", path_.size());
    } else {
      RCLCPP_INFO(get_logger(), "收到 %zu 个路径点，等待地图位姿后低速行驶。", path_.size());
    }
  }

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    if (state_ == State::Fault) return;
    const auto &position = message->pose.position;
    const auto &q = message->pose.orientation;
    if (message->header.frame_id != "map" || !std::isfinite(position.x) ||
        !std::isfinite(position.y)) {
      stop_with_fault("机器人地图位姿无效");
      return;
    }
    try {
      const auto rpy = course_bot_control::quaternion_to_rpy(q.x, q.y, q.z, q.w);
      current_pose_ = {position.x, position.y, rpy.yaw};
      have_pose_ = true;
      last_pose_time_ = Clock::now();
    } catch (const std::exception &error) {
      stop_with_fault(error.what());
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message) {
    if (state_ == State::Fault) return;
    const auto &q = message->pose.pose.orientation;
    try {
      const auto rpy = course_bot_control::quaternion_to_rpy(q.x, q.y, q.z, q.w);
      if (std::abs(rpy.roll) > 20.0 * course_bot_control::kPi / 180.0 ||
          std::abs(rpy.pitch) > 20.0 * course_bot_control::kPi / 180.0) {
        stop_with_fault("车体倾斜超过 20°");
        return;
      }
      have_odom_ = true;
      last_odom_time_ = Clock::now();
    } catch (const std::exception &error) {
      stop_with_fault(error.what());
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message) {
    if (state_ == State::Fault) return;
    if (!std::isfinite(message->angle_min) || !std::isfinite(message->angle_increment) ||
        message->angle_increment <= 0.0 || !std::isfinite(message->range_min) ||
        !std::isfinite(message->range_max) || message->range_max <= message->range_min) {
      stop_with_fault("/scan 的角度或量程参数无效");
      return;
    }

    const auto safety = course_bot_control::evaluate_laser_safety(
        message->ranges, message->angle_min, message->angle_increment,
        message->range_min, message->range_max, laser_front_half_angle_,
        laser_stop_distance_, laser_emergency_distance_);

    const bool was_front_blocked = laser_front_blocked_;
    const bool was_emergency_blocked = laser_emergency_blocked_;
    laser_front_blocked_ = safety.front_blocked;
    laser_emergency_blocked_ = safety.emergency_blocked;
    have_scan_ = true;
    last_scan_time_ = Clock::now();

    if ((!was_front_blocked && laser_front_blocked_) ||
        (!was_emergency_blocked && laser_emergency_blocked_)) {
      publish_stop();
      RCLCPP_WARN(get_logger(),
                  "激光安全停车：最近前方 %.2f 米、全周 %.2f 米；等待绕行路径。",
                  safety.nearest_front, safety.nearest_all);
    } else if ((was_front_blocked || was_emergency_blocked) &&
               !laser_front_blocked_ && !laser_emergency_blocked_) {
      RCLCPP_INFO(get_logger(), "激光安全区域已恢复，可继续执行规划路径。");
    }
  }

  void control_step() {
    if (state_ == State::Fault || state_ == State::Done) {
      publish_stop();
      return;
    }
    if (waiting_for_sensors_) {
      publish_stop();
      const auto now = Clock::now();
      const bool sensors_fresh = have_pose_ && have_odom_ && have_scan_ &&
          std::chrono::duration<double>(now - last_pose_time_).count() <= 1.0 &&
          std::chrono::duration<double>(now - last_odom_time_).count() <= 1.0 &&
          std::chrono::duration<double>(now - last_scan_time_).count() <= 1.0;
      if (sensors_fresh) {
        waiting_for_sensors_ = false;
        request_replan("位姿、里程计和激光数据已恢复");
      }
      return;
    }
    if (path_.empty() || !have_pose_ || !have_odom_ || !have_scan_) {
      publish_stop();
      return;
    }

    const auto now = Clock::now();
    if (std::chrono::duration<double>(now - last_pose_time_).count() > 1.0) {
      wait_for_sensors("超过 1 秒未收到机器人位姿");
      return;
    }
    if (std::chrono::duration<double>(now - last_odom_time_).count() > 1.0) {
      wait_for_sensors("超过 1 秒未收到 /odom");
      return;
    }
    if (std::chrono::duration<double>(now - last_scan_time_).count() > 1.0) {
      wait_for_sensors("超过 1 秒未收到 /scan，无法保证近距离安全");
      return;
    }
    if (std::chrono::duration<double>(now - last_path_time_).count() > 3.0) {
      stop_or_request_replan("超过 3 秒未收到规划路径，规划节点可能已退出");
      return;
    }
    if (state_ == State::Waiting) {
      const auto current = course_bot_control::PlanarPoint{current_pose_.x, current_pose_.y};
      if (point_distance(current, path_.front()) > 0.25) {
        stop_or_request_replan("机器人离路径起点超过 0.25 米");
        return;
      }
      state_ = State::Following;
      started_at_ = now;
      RCLCPP_INFO(get_logger(), "开始跟踪路径，最大线速度 %.2f 米/秒。", max_linear_);
    }
    if (std::chrono::duration<double>(now - started_at_).count() > 360.0) {
      stop_with_fault("路径跟踪超过 6 分钟，安全停车");
      return;
    }

    const auto current = course_bot_control::PlanarPoint{current_pose_.x, current_pose_.y};
    double nearest_distance = point_distance(current, path_.front());
    for (const auto &point : path_) {
      nearest_distance = std::min(nearest_distance, point_distance(current, point));
    }
    if (nearest_distance > 0.30) {
      stop_or_request_replan("机器人偏离规划路线超过 0.30 米");
      return;
    }

    // 靠近一个路径点后前往下一个；最后一个点用单独的终点容差。
    while (target_index_ < path_.size()) {
      const double tolerance = target_index_ + 1 == path_.size()
                                   ? kGoalTolerance : kWaypointTolerance;
      if (course_bot_control::distance_to(current_pose_, path_[target_index_]) > tolerance) {
        break;
      }
      ++target_index_;
    }
    if (target_index_ == path_.size()) {
      state_ = State::Done;
      publish_stop();
      if (recoverable_path_errors_) {
        geometry_msgs::msg::PoseStamped completed;
        completed.header.frame_id = "map";
        completed.header.stamp = this->now();
        completed.pose.position.x = path_.back().x;
        completed.pose.position.y = path_.back().y;
        completed.pose.orientation.w = 1.0;
        goal_reached_publisher_->publish(completed);
      }
      RCLCPP_INFO(get_logger(),
                  "已到达目标点，机器人停车并等待下一条新路径；可在 RViz 点击新目标。");
      return;
    }

    const auto command = course_bot_control::steer_to(
        current_pose_, path_[target_index_], max_linear_, max_angular_);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.linear_x;
    twist.angular.z = command.angular_z;
    if (laser_emergency_blocked_) {
      // 全周极近距离内禁止任何运动，避免原地旋转时车身侧面碰撞。
      twist = geometry_msgs::msg::Twist{};
    } else if (laser_front_blocked_ && twist.linear.x > 0.0) {
      // 禁止继续接近前方物体，但保留原地旋转能力，以便转向新规划的绕行路线。
      twist.linear.x = 0.0;
    }
    cmd_publisher_->publish(twist);
  }

  void publish_stop() { cmd_publisher_->publish(geometry_msgs::msg::Twist{}); }

  void wait_for_sensors(const std::string &reason) {
    if (!recoverable_path_errors_) {
      stop_with_fault(reason);
      return;
    }
    publish_stop();
    path_.clear();
    target_index_ = 0;
    state_ = State::Waiting;
    waiting_for_sensors_ = true;
    RCLCPP_WARN(get_logger(), "安全停车：%s；等待传感器恢复后再规划。", reason.c_str());
  }

  void request_replan(const std::string &reason) {
    publish_stop();
    path_.clear();
    target_index_ = 0;
    state_ = State::Waiting;
    geometry_msgs::msg::PoseStamped request;
    request.header.frame_id = "map";
    request.header.stamp = this->now();
    request.pose.orientation.w = 1.0;
    replan_publisher_->publish(request);
    RCLCPP_WARN(get_logger(), "安全停车：%s；等待从当前位置重新规划。", reason.c_str());
  }

  void stop_or_request_replan(const std::string &reason) {
    if (!recoverable_path_errors_) {
      stop_with_fault(reason);
      return;
    }
    request_replan(reason);
  }

  void stop_with_fault(const std::string &reason) {
    if (state_ == State::Fault || state_ == State::Done) return;
    state_ = State::Fault;
    publish_stop();
    RCLCPP_ERROR(get_logger(), "安全停车：%s。请检查后重新启动节点。", reason.c_str());
  }

  State state_{State::Waiting};
  double max_linear_{0.10};
  double max_angular_{0.30};
  bool recoverable_path_errors_{false};
  bool waiting_for_sensors_{false};
  double laser_stop_distance_{0.40};
  double laser_emergency_distance_{0.25};
  double laser_front_half_angle_{0.52};
  std::vector<course_bot_control::PlanarPoint> path_;
  std::size_t target_index_{0};
  bool have_pose_{false};
  bool have_odom_{false};
  bool have_scan_{false};
  bool laser_front_blocked_{false};
  bool laser_emergency_blocked_{false};
  course_bot_control::PlanarPose current_pose_{};
  Clock::time_point last_pose_time_{};
  Clock::time_point last_odom_time_{};
  Clock::time_point last_scan_time_{};
  Clock::time_point last_path_time_{};
  Clock::time_point started_at_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr replan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_reached_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  // 与 basic_motion 一样，Ctrl+C 时先保留 ROS 通信，让零速度有机会送达。
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  const auto previous_sigint = std::signal(SIGINT, request_stop);
  const auto previous_sigterm = std::signal(SIGTERM, request_stop);
  std::shared_ptr<PathFollower> node;
  rclcpp::executors::SingleThreadedExecutor executor;
  bool attached = false;
  int result = 0;
  try {
    node = std::make_shared<PathFollower>();
    executor.add_node(node);
    attached = true;
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } catch (const std::exception &error) {
    std::cerr << "路径跟踪节点异常：" << error.what() << '\n';
    result = 1;
  }
  if (node && rclcpp::ok()) node->stop_before_exit();
  if (attached) executor.remove_node(node);
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  std::signal(SIGINT, previous_sigint);
  std::signal(SIGTERM, previous_sigterm);
  return result;
}
