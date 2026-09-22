#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "course_bot_control/laser_safety.hpp"

namespace {
void expect(bool condition, const std::string &reason) {
  if (!condition) throw std::runtime_error(reason);
}
}  // namespace

int main() {
  try {
    const double inf = std::numeric_limits<double>::infinity();

    // 五束激光从 -90° 到 +90°；中间一束正对车头。
    const auto front = course_bot_control::evaluate_laser_safety(
        {static_cast<float>(inf), 2.0F, 0.35F, 2.0F, static_cast<float>(inf)},
        -course_bot_control::kPi / 2.0, course_bot_control::kPi / 4.0,
        0.12, 12.0, 0.52, 0.40, 0.25);
    expect(front.front_blocked, "0.35 m front obstacle should block forward motion");
    expect(!front.emergency_blocked, "0.35 m obstacle should still allow turning");

    // 极近障碍位于侧面，也必须禁止原地旋转。
    const auto side = course_bot_control::evaluate_laser_safety(
        {0.20F, 2.0F, 2.0F, 2.0F, 2.0F},
        -course_bot_control::kPi / 2.0, course_bot_control::kPi / 4.0,
        0.12, 12.0, 0.52, 0.40, 0.25);
    expect(!side.front_blocked, "side obstacle must not be classified as front obstacle");
    expect(side.emergency_blocked, "0.20 m side obstacle should stop every motion");

    const auto clear = course_bot_control::evaluate_laser_safety(
        {static_cast<float>(inf), 3.0F, 2.0F, 3.0F, static_cast<float>(inf)},
        -course_bot_control::kPi / 2.0, course_bot_control::kPi / 4.0,
        0.12, 12.0, 0.52, 0.40, 0.25);
    expect(!clear.front_blocked && !clear.emergency_blocked,
           "clear scan should permit normal path following");

    std::cout << "Laser safety C++ tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Laser safety test failed: " << error.what() << '\n';
    return 1;
  }
}
