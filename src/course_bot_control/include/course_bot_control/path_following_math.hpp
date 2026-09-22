#ifndef COURSE_BOT_CONTROL__PATH_FOLLOWING_MATH_HPP_
#define COURSE_BOT_CONTROL__PATH_FOLLOWING_MATH_HPP_

#include <algorithm>
#include <cmath>

#include "course_bot_control/motion_math.hpp"

namespace course_bot_control {

struct PlanarPose {
  double x{};
  double y{};
  double yaw{};
};

struct PlanarPoint {
  double x{};
  double y{};
};

struct Velocity2D {
  double linear_x{};
  double angular_z{};
};

inline double distance_to(PlanarPose robot, PlanarPoint target) {
  return std::hypot(target.x - robot.x, target.y - robot.y);
}

// 大角度时先原地转向；对准后低速前进并持续修正航向。
// 是否已到达路径点由调用方按容差判断，此函数只计算运动指令。
inline Velocity2D steer_to(PlanarPose robot, PlanarPoint target,
                           double max_linear, double max_angular) {
  const double distance = distance_to(robot, target);
  const double desired_yaw = std::atan2(target.y - robot.y, target.x - robot.x);
  const double yaw_error = wrap_angle(desired_yaw - robot.yaw);
  const double turn = std::clamp(1.5 * yaw_error, -max_angular, max_angular);
  if (std::abs(yaw_error) > 0.30) {
    return {0.0, turn};
  }
  const double forward = std::min(max_linear, std::max(0.04, 0.8 * distance));
  return {forward, turn};
}

}  // namespace course_bot_control

#endif  // COURSE_BOT_CONTROL__PATH_FOLLOWING_MATH_HPP_
