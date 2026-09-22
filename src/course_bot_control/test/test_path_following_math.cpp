#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "course_bot_control/path_following_math.hpp"

namespace {
void expect(bool condition, const std::string &reason) {
  if (!condition) throw std::runtime_error(reason);
}

void expect_near(double actual, double expected, const std::string &reason) {
  expect(std::abs(actual - expected) < 1e-9, reason);
}
}  // namespace

int main() {
  try {
    using course_bot_control::PlanarPoint;
    using course_bot_control::PlanarPose;
    const PlanarPose robot{0.0, 0.0, 0.0};
    const auto straight = course_bot_control::steer_to(robot, {1.0, 0.0}, 0.12, 0.35);
    expect_near(straight.linear_x, 0.12, "straight speed limit");
    expect_near(straight.angular_z, 0.0, "straight turn rate");

    const auto turn = course_bot_control::steer_to(robot, {0.0, 1.0}, 0.12, 0.35);
    expect_near(turn.linear_x, 0.0, "large heading error must stop forward motion");
    expect_near(turn.angular_z, 0.35, "turn rate limit");

    const auto gentle = course_bot_control::steer_to(robot, {1.0, 0.1}, 0.12, 0.35);
    expect(gentle.linear_x > 0.0, "small heading error should allow forward motion");
    expect(gentle.angular_z > 0.0, "small heading error should steer left");

    const auto nearby = course_bot_control::steer_to(robot, {0.1, 0.0}, 0.12, 0.35);
    expect_near(nearby.linear_x, 0.08, "nearby point should slow down");
    expect_near(course_bot_control::distance_to(robot, PlanarPoint{0.3, 0.4}), 0.5,
                "distance calculation");

    // 理想运动学小实验：以 0.1 秒步长模拟一条含 90° 转弯的短路径。
    const std::vector<PlanarPoint> path{{0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0},
                                        {0.4, 0.2}, {0.4, 0.4}};
    PlanarPose moving{0.0, 0.0, 0.0};
    std::size_t target_index = 0;
    for (int step = 0; step < 1500 && target_index < path.size(); ++step) {
      while (target_index < path.size() &&
             course_bot_control::distance_to(moving, path[target_index]) <= 0.13) {
        ++target_index;
      }
      if (target_index == path.size()) break;
      const auto command = course_bot_control::steer_to(
          moving, path[target_index], 0.10, 0.30);
      moving.x += command.linear_x * std::cos(moving.yaw) * 0.1;
      moving.y += command.linear_x * std::sin(moving.yaw) * 0.1;
      moving.yaw = course_bot_control::wrap_angle(moving.yaw + command.angular_z * 0.1);
    }
    expect(target_index == path.size(), "simulated robot did not finish the turn path");
    expect(course_bot_control::distance_to(moving, path.back()) <= 0.13,
           "simulated robot stopped too far from goal");
    std::cout << "Path following math C++ tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Path following math test failed: " << error.what() << '\n';
    return 1;
  }
}
