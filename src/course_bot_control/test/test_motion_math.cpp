#include <cmath>
#include <iostream>
#include <stdexcept>

#include "course_bot_control/motion_math.hpp"

namespace {
void check_close(double actual, double expected, const char *name) {
  if (std::abs(actual - expected) > 1e-6) {
    throw std::runtime_error(name);
  }
}
}  // namespace

int main() {
  using course_bot_control::kPi;
  using course_bot_control::quaternion_to_rpy;
  using course_bot_control::relative_displacement;
  using course_bot_control::wrap_angle;

  check_close(wrap_angle(kPi + 0.1), -kPi + 0.1, "角度跨越边界失败");
  const double q = std::sqrt(0.5);
  const auto rpy = quaternion_to_rpy(0.0, 0.0, q, q);
  check_close(rpy.yaw, kPi / 2.0, "四元数转换失败");
  const auto [forward, sideways] = relative_displacement(0.0, 0.0, kPi / 2.0, 0.0, 1.0);
  check_close(forward, 1.0, "前进位移投影失败");
  check_close(sideways, 0.0, "横向位移投影失败");

  bool rejected = false;
  try {
    quaternion_to_rpy(0.0, 0.0, 0.0, 0.0);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  if (!rejected) throw std::runtime_error("无效四元数未被拒绝");

  std::cout << "motion_math C++ tests passed\n";
  return 0;
}
