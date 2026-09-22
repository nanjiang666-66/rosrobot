#pragma once

namespace course_bot_planner {

struct Pose2D {
  double x{};
  double y{};
  double yaw{};
};

// 第一次里程计到来时，小车必须还在已知 Gazebo 生成位置。
// 本类求固定的 map -> odom 变换，之后把 odom 位姿换算到地图坐标。
class OdomAnchor {
public:
  OdomAnchor(Pose2D first_odom, Pose2D initial_world);

  Pose2D map_to_odom_transform() const noexcept { return map_to_odom_; }
  Pose2D odom_pose_in_map(Pose2D odom) const noexcept;

private:
  Pose2D map_to_odom_{};
};

}  // namespace course_bot_planner
