#include "course_bot_planner/course_world_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace course_bot_planner {
namespace {

// 本文件的几何参数对应 course_bot_gazebo/worlds/course_obstacles.world。
// 修改 Gazebo 世界里的障碍物后，必须同步修改这里并重新运行地图测试。
double distance_to_box(WorldPoint point, WorldPoint center,
                       double half_x, double half_y, double yaw) {
  const double dx = point.x - center.x;
  const double dy = point.y - center.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double outside_x = std::max(std::abs(local_x) - half_x, 0.0);
  const double outside_y = std::max(std::abs(local_y) - half_y, 0.0);
  return std::hypot(outside_x, outside_y);
}

bool near_obstacle(WorldPoint point, double clearance,
                   const std::vector<StaticObstacle2D> &obstacles) {
  for (const auto &obstacle : obstacles) {
    if (obstacle.shape == ObstacleShape::Box) {
      if (distance_to_box(point, obstacle.center, obstacle.size_x / 2.0,
                          obstacle.size_y / 2.0, obstacle.yaw) <= clearance) {
        return true;
      }
    } else if (std::hypot(point.x - obstacle.center.x,
                          point.y - obstacle.center.y) <=
               obstacle.radius + clearance) {
      // 竖直圆柱和球体在地面上的投影都是圆。
      return true;
    }
  }
  return false;
}

std::vector<StaticObstacle2D> builtin_obstacles() {
  // 仅作为显式调试备用；正常运行由 sdf_world_loader 读取世界文件。
  return {
      {"obstacle_box_1", ObstacleShape::Box, {-1.0, -1.2}, 0.0, 1.0, 1.0, 0.0},
      {"obstacle_box_2", ObstacleShape::Box, {1.5, 1.0}, 0.35, 1.4, 0.7, 0.0},
      {"obstacle_cylinder", ObstacleShape::Cylinder, {3.0, -1.5}, 0.0, 0.0, 0.0,
       0.55},
      {"obstacle_sphere", ObstacleShape::Sphere, {0.5, -2.7}, 0.0, 0.0, 0.0, 0.45},
  };
}

}  // namespace

CourseWorldMap::CourseWorldMap() : CourseWorldMap(builtin_obstacles()) {}

CourseWorldMap::CourseWorldMap(const std::vector<StaticObstacle2D> &obstacles)
    : grid(kWidth, kHeight) {
  // 机器人外接圆 + 安全余量 + 半个栅格对角线。
  // 最后一项保证：即使车位于该格的边缘，也不会贴到障碍物。
  const double cell_half_diagonal = 0.5 * std::sqrt(2.0) * kResolution;
  const double clearance = kRobotRadius + kSafetyMargin + cell_half_diagonal;
  const double max_x = kOriginX + kWidth * kResolution;
  const double max_y = kOriginY + kHeight * kResolution;

  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const Cell cell{x, y};
      const WorldPoint center = cell_center(cell);
      // 地图之外未建立障碍信息；边界也留出足够的车身通行余量。
      const bool near_boundary = center.x - kOriginX <= clearance ||
                                 max_x - center.x <= clearance ||
                                 center.y - kOriginY <= clearance ||
                                 max_y - center.y <= clearance;
      if (near_boundary || near_obstacle(center, clearance, obstacles)) {
        grid.set_blocked(cell);
      }
    }
  }
}

CourseWorldMap::CourseWorldMap(int width, int height, double resolution,
                               double origin_x, double origin_y)
    : grid(width, height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y) {
  if (!(std::isfinite(resolution_) && resolution_ > 0.0)) {
    throw std::invalid_argument("map resolution must be positive and finite");
  }
  if (!std::isfinite(origin_x_) || !std::isfinite(origin_y_)) {
    throw std::invalid_argument("map origin must be finite");
  }
}

std::optional<Cell> CourseWorldMap::world_to_cell(WorldPoint point) const noexcept {
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
  const double relative_x = (point.x - origin_x_) / resolution_;
  const double relative_y = (point.y - origin_y_) / resolution_;
  if (relative_x < 0.0 || relative_x >= grid.width() ||
      relative_y < 0.0 || relative_y >= grid.height()) {
    return std::nullopt;
  }
  return Cell{static_cast<int>(std::floor(relative_x)),
              static_cast<int>(std::floor(relative_y))};
}

WorldPoint CourseWorldMap::cell_center(Cell cell) const {
  if (!grid.in_bounds(cell)) throw std::out_of_range("cell lies outside course world map");
  return {origin_x_ + (cell.x + 0.5) * resolution_,
          origin_y_ + (cell.y + 0.5) * resolution_};
}

}  // namespace course_bot_planner
