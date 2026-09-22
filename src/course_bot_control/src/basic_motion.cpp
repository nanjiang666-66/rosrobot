// 第 3 阶段 C++ 控制节点：依靠 /odom 反馈完成定距直行、左转和矩形轨迹。
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cctype>
#include <cstddef>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "course_bot_control/motion_math.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;

// 信号处理器只修改一个标志，不调用 ROS API；主循环负责安全停车。
void request_stop(int) { stop_requested = 1; }
}  // namespace

using course_bot_control::kPi;

class BasicMotion final : public rclcpp::Node {
public:
  BasicMotion() : Node("basic_motion") {
    mode_ = declare_parameter<std::string>("mode", "straight");
    target_distance_ = declare_parameter<double>("target_distance", 0.5);
    linear_speed_ = declare_parameter<double>("linear_speed", 0.15);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.4);
    long_side_ = declare_parameter<double>("long_side", 0.8);
    short_side_ = declare_parameter<double>("short_side", 0.5);

    std::transform(mode_.begin(), mode_.end(), mode_.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (mode_ != "straight" && mode_ != "rotate" && mode_ != "rectangle") {
      throw std::invalid_argument("mode 只能是 straight、rotate 或 rectangle");
    }
    for (const auto &item : std::array<std::pair<const char *, double>, 3>{{
             {"target_distance", target_distance_}, {"long_side", long_side_},
             {"short_side", short_side_}}}) {
      if (!(item.second > 0.0 && item.second <= 1.0)) {
        throw std::invalid_argument(std::string(item.first) + " 必须大于 0 且不超过 1.0 米");
      }
    }
    if (!(linear_speed_ > 0.0 && linear_speed_ <= 0.3)) {
      throw std::invalid_argument("linear_speed 必须大于 0 且不超过 0.3 米/秒");
    }
    if (!(angular_speed_ > 0.0 && angular_speed_ <= 0.6)) {
      throw std::invalid_argument("angular_speed 必须大于 0 且不超过 0.6 弧度/秒");
    }
    rectangle_sides_ = {long_side_, short_side_, long_side_, short_side_};

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&BasicMotion::on_odom, this, std::placeholders::_1));
    control_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                       std::bind(&BasicMotion::control_step, this));
    RCLCPP_INFO(get_logger(), "模式 %s：等待 /odom，收到反馈后开始。", mode_.c_str());
  }

  void stop_before_exit() {
    // Ctrl+C 时通信仍有效；连续发零速度，避免单条消息在退出瞬间丢失。
    RCLCPP_INFO(get_logger(), "收到退出信号，正在发送零速度停车命令。");
    for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
      publish_stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

private:
  enum class State { WaitOdom, Drive, Turn, Done, Fault };
  using Clock = std::chrono::steady_clock;

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto &position = msg->pose.pose.position;
    const auto &orientation = msg->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
      stop_with_fault("里程计位置不是有效数字");
      return;
    }
    course_bot_control::Rpy rpy{};
    try {
      rpy = course_bot_control::quaternion_to_rpy(
          orientation.x, orientation.y, orientation.z, orientation.w);
    } catch (const std::exception &error) {
      stop_with_fault(error.what());
      return;
    }
    if (std::abs(rpy.roll) > 20.0 * kPi / 180.0 ||
        std::abs(rpy.pitch) > 20.0 * kPi / 180.0) {
      stop_with_fault("车体倾斜超过 20°，请检查四轮支撑");
      return;
    }

    x_ = position.x;
    y_ = position.y;
    last_odom_time_ = Clock::now();
    const double yaw = rpy.yaw;
    if (state_ == State::Turn && have_yaw_) {
      // 航向跨越 +π/-π 时，仅累计真实的小角度变化。
      turn_progress_ += course_bot_control::wrap_angle(yaw - previous_yaw_);
    }
    yaw_ = yaw;
    previous_yaw_ = yaw;
    have_yaw_ = true;

    if (state_ == State::WaitOdom) {
      if (mode_ == "rotate") {
        begin_turn();
      } else {
        begin_drive(mode_ == "straight" ? target_distance_ : rectangle_sides_[0]);
      }
    }
  }

  void begin_drive(double distance) {
    // 每段重新记录起点，避免矩形前几段的累计误差影响这一段。
    drive_origin_x_ = x_;
    drive_origin_y_ = y_;
    drive_heading_ = yaw_;
    drive_target_ = distance;
    state_started_at_ = Clock::now();
    state_ = State::Drive;
    RCLCPP_INFO(get_logger(), "直行第 %zu 段，目标 %.2f 米", side_index_ + 1, distance);
  }

  void begin_turn() {
    turn_progress_ = 0.0;
    previous_yaw_ = yaw_;
    state_started_at_ = Clock::now();
    state_ = State::Turn;
    RCLCPP_INFO(get_logger(), "开始原地左转约 90°");
  }

  void control_step() {
    if (state_ == State::WaitOdom) return;  // 没有里程计时禁止运动。
    if (state_ == State::Done || state_ == State::Fault) {
      publish_stop();
      return;
    }

    const auto now = Clock::now();
    const double odom_age = std::chrono::duration<double>(now - last_odom_time_).count();
    if (odom_age > 1.0) {
      stop_with_fault("超过 1 秒未收到 /odom");
      return;
    }
    const double state_age = std::chrono::duration<double>(now - state_started_at_).count();
    if (state_ == State::Drive) {
      const double limit = std::max(10.0, drive_target_ / linear_speed_ * 2.0 + 3.0);
      if (state_age > limit) {
        stop_with_fault("直行超时，机器人可能被卡住");
        return;
      }
      drive_step();
    } else if (state_ == State::Turn) {
      const double limit = std::max(10.0, turn_target_ / angular_speed_ * 2.0 + 3.0);
      if (state_age > limit) {
        stop_with_fault("转向超时，请检查左右轮方向");
        return;
      }
      turn_step();
    }
  }

  void drive_step() {
    const auto [forward, sideways] = course_bot_control::relative_displacement(
        drive_origin_x_, drive_origin_y_, drive_heading_, x_, y_);
    if (std::abs(sideways) > 0.20) {
      stop_with_fault("直行横向偏移超过 0.20 米");
      return;
    }
    const double remaining = drive_target_ - forward;
    if (remaining <= 0.02) {
      publish_stop();
      RCLCPP_INFO(get_logger(), "第 %zu 段完成，前进 %.2f 米", side_index_ + 1, forward);
      if (mode_ == "rectangle") begin_turn();
      else finish();
      return;
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = std::min(linear_speed_, std::max(0.05, remaining * 0.8));
    cmd_pub_->publish(command);
  }

  void turn_step() {
    const double remaining = turn_target_ - turn_progress_;
    if (remaining <= 3.0 * kPi / 180.0) {
      publish_stop();
      RCLCPP_INFO(get_logger(), "左转完成，累计 %.1f°", turn_progress_ * 180.0 / kPi);
      if (mode_ == "rectangle") {
        ++side_index_;
        if (side_index_ < rectangle_sides_.size()) begin_drive(rectangle_sides_[side_index_]);
        else finish();
      } else {
        finish();
      }
      return;
    }

    geometry_msgs::msg::Twist command;
    command.angular.z = std::min(angular_speed_, std::max(0.12, remaining * 1.5));
    cmd_pub_->publish(command);
  }

  void publish_stop() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }

  void finish() {
    state_ = State::Done;
    publish_stop();
    RCLCPP_INFO(get_logger(), "任务完成，机器人已停车。按 Ctrl+C 退出。");
  }

  void stop_with_fault(const std::string &reason) {
    if (state_ == State::Fault || state_ == State::Done) return;
    state_ = State::Fault;
    publish_stop();
    RCLCPP_ERROR(get_logger(), "安全停车：%s", reason.c_str());
  }

  State state_{State::WaitOdom};
  std::string mode_;
  double target_distance_{0.5};
  double linear_speed_{0.15};
  double angular_speed_{0.4};
  double long_side_{0.8};
  double short_side_{0.5};
  std::array<double, 4> rectangle_sides_{};
  std::size_t side_index_{0};
  double x_{0.0}, y_{0.0}, yaw_{0.0}, previous_yaw_{0.0};
  bool have_yaw_{false};
  Clock::time_point last_odom_time_{};
  Clock::time_point state_started_at_{};
  double drive_origin_x_{0.0}, drive_origin_y_{0.0}, drive_heading_{0.0};
  double drive_target_{0.0};
  double turn_progress_{0.0};
  const double turn_target_{kPi / 2.0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
  // 关闭 rclcpp 默认的“收到 Ctrl+C 立刻 shutdown”，先保留通信以便发停车命令。
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  const auto previous_sigint = std::signal(SIGINT, request_stop);
  const auto previous_sigterm = std::signal(SIGTERM, request_stop);
  std::shared_ptr<BasicMotion> node;
  rclcpp::executors::SingleThreadedExecutor executor;
  bool attached = false;
  int result = 0;
  try {
    node = std::make_shared<BasicMotion>();
    executor.add_node(node);
    attached = true;
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } catch (const std::exception &error) {
    std::cerr << "控制节点异常：" << error.what() << '\n';
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
