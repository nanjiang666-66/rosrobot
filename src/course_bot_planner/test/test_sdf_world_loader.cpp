#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "course_bot_planner/sdf_world_loader.hpp"

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(1);
  }
}

bool near(double actual, double expected) {
  return std::abs(actual - expected) < 1e-9;
}

}  // namespace

int main() {
  const auto result =
      course_bot_planner::load_static_obstacles_from_sdf(TEST_WORLD_FILE);
  require(result.obstacles.size() == 3, "应读取 box、cylinder、sphere 三个碰撞体");
  require(result.skipped_dynamic_models == 1, "应跳过一个动态模型");
  require(result.skipped_horizontal_planes == 1,
          "应按几何类型跳过 ground_plane_0 水平地面");

  const auto &box = result.obstacles.at(0);
  require(box.shape == course_bot_planner::ObstacleShape::Box, "第一个应为 Box");
  require(near(box.center.x, 1.5) && near(box.center.y, 2.0),
          "应合成 model 与 collision 位姿");
  require(near(box.size_x, 1.2) && near(box.size_y, 0.8), "Box 尺寸不正确");

  const auto &cylinder = result.obstacles.at(1);
  require(cylinder.shape == course_bot_planner::ObstacleShape::Cylinder,
          "第二个应为 Cylinder");
  require(near(cylinder.radius, 0.4), "Cylinder 半径不正确");

  const auto &sphere = result.obstacles.at(2);
  require(sphere.shape == course_bot_planner::ObstacleShape::Sphere,
          "第三个应为 Sphere");
  require(near(sphere.center.x, 0.25) && near(sphere.center.y, -0.5),
          "Sphere 位置不正确");
  require(near(sphere.radius, 0.45), "Sphere 半径不正确");

  course_bot_planner::CourseWorldMap map(result.obstacles);
  const auto sphere_cell = map.world_to_cell(sphere.center);
  require(sphere_cell.has_value() && !map.grid.traversable(*sphere_cell),
          "Sphere 中心对应栅格必须被占据");
  std::cout << "SDF loader tests passed\n";
  return 0;
}
