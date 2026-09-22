#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "course_bot_planner/course_world_map.hpp"

namespace course_bot_planner {

struct SdfWorldLoadResult {
  std::vector<StaticObstacle2D> obstacles;
  std::size_t skipped_dynamic_models{};
  std::size_t skipped_horizontal_planes{};
};

// 使用 libsdformat 读取世界文件。遇到损坏文件、无法解析的位姿，或静态模型
// 使用未支持的碰撞形状时抛出异常，避免发布一张漏掉障碍物的危险地图。
SdfWorldLoadResult load_static_obstacles_from_sdf(const std::string &world_file);

}  // namespace course_bot_planner
