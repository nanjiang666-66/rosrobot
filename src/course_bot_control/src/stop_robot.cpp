// 应急停车：控制节点异常退出时，连续覆盖 Gazebo 中保留的旧速度命令。
#include <chrono>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("course_bot_emergency_stop");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 持续约 2 秒：给 DDS 发现订阅者的时间，并确保零速度送达。
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(geometry_msgs::msg::Twist{});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_INFO(node->get_logger(), "已连续发送零速度停车命令。");
  publisher.reset();
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
