#ifndef COURSE_BOT_PLANNER__REMAINING_PATH_HPP_
#define COURSE_BOT_PLANNER__REMAINING_PATH_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

#include "course_bot_planner/course_world_map.hpp"

namespace course_bot_planner {

// 地图更新只需检查尚未行驶的路径。障碍膨胀已由调用方写入 grid。
inline bool remaining_path_blocked(const CourseWorldMap &map,
                                   const std::vector<WorldPoint> &path,
                                   WorldPoint robot) {
  if (path.empty()) return true;
  std::size_t nearest = 0;
  double best = std::hypot(path[0].x - robot.x, path[0].y - robot.y);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double distance = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  // 最近路径点本身也检查；略微退后一步可覆盖当前线段的安全边界。
  const std::size_t first = nearest > 0 ? nearest - 1 : 0;
  for (std::size_t i = first; i < path.size(); ++i) {
    const auto cell = map.world_to_cell(path[i]);
    if (!cell || !map.grid.traversable(*cell)) return true;
    if (i == first) continue;
    const auto previous = map.world_to_cell(path[i - 1]);
    if (!previous) return true;
    const int dx = cell->x - previous->x;
    const int dy = cell->y - previous->y;
    // A* 禁止斜穿墙角；地图变化后也要维持这条约束。
    if (std::abs(dx) == 1 && std::abs(dy) == 1 &&
        (!map.grid.traversable({previous->x + dx, previous->y}) ||
         !map.grid.traversable({previous->x, previous->y + dy}))) {
      return true;
    }
  }
  return false;
}

}  // namespace course_bot_planner

#endif  // COURSE_BOT_PLANNER__REMAINING_PATH_HPP_
