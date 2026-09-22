#ifndef COURSE_BOT_CONTROL__LASER_SAFETY_HPP_
#define COURSE_BOT_CONTROL__LASER_SAFETY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "course_bot_control/motion_math.hpp"

namespace course_bot_control {

// 与 ROS 消息类型无关的雷达安全判定，便于单独理解和测试。
struct LaserSafetyObservation {
  double nearest_all{std::numeric_limits<double>::infinity()};
  double nearest_front{std::numeric_limits<double>::infinity()};
  bool front_blocked{false};
  bool emergency_blocked{false};
};

inline LaserSafetyObservation evaluate_laser_safety(
    const std::vector<float> &ranges, double angle_min, double angle_increment,
    double range_min, double range_max, double front_half_angle,
    double stop_distance, double emergency_distance) {
  LaserSafetyObservation result;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double range = ranges[i];
    // 激光的 +inf 表示没有命中；超出传感器有效量程的数值也应忽略。
    if (!std::isfinite(range) || range < range_min || range > range_max) continue;
    result.nearest_all = std::min(result.nearest_all, range);
    const double angle = angle_min + static_cast<double>(i) * angle_increment;
    if (std::abs(wrap_angle(angle)) <= front_half_angle) {
      result.nearest_front = std::min(result.nearest_front, range);
    }
  }
  result.front_blocked = result.nearest_front <= stop_distance;
  result.emergency_blocked = result.nearest_all <= emergency_distance;
  return result;
}

}  // namespace course_bot_control

#endif  // COURSE_BOT_CONTROL__LASER_SAFETY_HPP_
