#include "course_bot_planner/odom_anchor.hpp"

#include <cmath>
#include <stdexcept>

namespace course_bot_planner {
namespace {
double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
}

OdomAnchor::OdomAnchor(Pose2D first_odom, Pose2D initial_world) {
  if (!std::isfinite(first_odom.x) || !std::isfinite(first_odom.y) ||
      !std::isfinite(first_odom.yaw) || !std::isfinite(initial_world.x) ||
      !std::isfinite(initial_world.y) || !std::isfinite(initial_world.yaw)) {
    throw std::invalid_argument("odom anchor poses must contain finite values");
  }
  // T_map_odom = T_map_base(initial) * inverse(T_odom_base(initial))。
  map_to_odom_.yaw = wrap_angle(initial_world.yaw - first_odom.yaw);
  const double cosine = std::cos(map_to_odom_.yaw);
  const double sine = std::sin(map_to_odom_.yaw);
  map_to_odom_.x = initial_world.x - cosine * first_odom.x + sine * first_odom.y;
  map_to_odom_.y = initial_world.y - sine * first_odom.x - cosine * first_odom.y;
}

Pose2D OdomAnchor::odom_pose_in_map(Pose2D odom) const noexcept {
  const double cosine = std::cos(map_to_odom_.yaw);
  const double sine = std::sin(map_to_odom_.yaw);
  return {map_to_odom_.x + cosine * odom.x - sine * odom.y,
          map_to_odom_.y + sine * odom.x + cosine * odom.y,
          wrap_angle(map_to_odom_.yaw + odom.yaw)};
}

}  // namespace course_bot_planner
