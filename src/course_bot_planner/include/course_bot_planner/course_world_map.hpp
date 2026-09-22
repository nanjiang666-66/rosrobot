#pragma once

#include <optional>
#include <string>
#include <vector>

#include "course_bot_planner/astar.hpp"

namespace course_bot_planner {

// Gazebo 平面上的米制位置；与 Cell 的整数栅格坐标刻意分开。
struct WorldPoint {
  double x{};
  double y{};
};

// 从 SDF 碰撞几何提取出的二维障碍物。地图只关心地面的投影。
enum class ObstacleShape { Box, Cylinder, Sphere };

struct StaticObstacle2D {
  std::string name;
  ObstacleShape shape{ObstacleShape::Box};
  WorldPoint center{};
  double yaw{};
  double size_x{};  // Box 的完整 X 尺寸。
  double size_y{};  // Box 的完整 Y 尺寸。
  double radius{};  // Cylinder / Sphere 的半径。
};

class CourseWorldMap {
public:
  static constexpr int kWidth = 50;
  static constexpr int kHeight = 50;
  static constexpr double kResolution = 0.20;  // 每格 20 cm。
  static constexpr double kOriginX = -5.0;
  static constexpr double kOriginY = -5.0;
  static constexpr double kRobotRadius = 0.31;  // 覆盖 0.50 m × 0.34 m 底盘的角点。
  static constexpr double kSafetyMargin = 0.10;

  CourseWorldMap();
  explicit CourseWorldMap(const std::vector<StaticObstacle2D> &obstacles);
  // SLAM 的 OccupancyGrid 尺寸会随探索范围变化，因此允许在运行时创建任意栅格。
  // 这个构造函数只创建空地图，调用方再根据 /map 数据设置障碍格。
  CourseWorldMap(int width, int height, double resolution,
                 double origin_x, double origin_y);

  std::optional<Cell> world_to_cell(WorldPoint point) const noexcept;
  WorldPoint cell_center(Cell cell) const;

  double resolution() const noexcept { return resolution_; }
  double origin_x() const noexcept { return origin_x_; }
  double origin_y() const noexcept { return origin_y_; }

  Grid grid;

private:
  double resolution_{kResolution};
  double origin_x_{kOriginX};
  double origin_y_{kOriginY};
};

// 起点与 Gazebo XML 中的生成位置相同；终点是当前世界中的安全空地。
inline constexpr WorldPoint kDefaultStartWorld{-4.0, -3.0};
inline constexpr WorldPoint kDefaultGoalWorld{4.0, 3.0};

}  // namespace course_bot_planner
