#ifndef COURSE_BOT_CONTROL__MOTION_MATH_HPP_
#define COURSE_BOT_CONTROL__MOTION_MATH_HPP_

// 独立于 ROS 的计算函数，便于用普通 C++ 编译器验证。
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace course_bot_control {
inline constexpr double kPi = 3.14159265358979323846;

struct Rpy {
  double roll;
  double pitch;
  double yaw;
};

inline double wrap_angle(double angle) {
  double result = std::fmod(angle + kPi, 2.0 * kPi);
  if (result < 0.0) result += 2.0 * kPi;
  return result - kPi;
}

inline Rpy quaternion_to_rpy(double x, double y, double z, double w) {
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::runtime_error("无效的里程计姿态四元数");
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch_sine = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
  const double pitch = std::asin(pitch_sine);
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return {roll, pitch, yaw};
}

inline std::pair<double, double> relative_displacement(
    double start_x, double start_y, double heading, double x, double y) {
  const double dx = x - start_x;
  const double dy = y - start_y;
  const double forward = dx * std::cos(heading) + dy * std::sin(heading);
  const double sideways = -dx * std::sin(heading) + dy * std::cos(heading);
  return {forward, sideways};
}
}  // namespace course_bot_control

#endif  // COURSE_BOT_CONTROL__MOTION_MATH_HPP_
