#include <cassert>
#include <vector>

#include "course_bot_planner/remaining_path.hpp"

int main() {
  using course_bot_planner::CourseWorldMap;
  using course_bot_planner::WorldPoint;
  CourseWorldMap map(10, 10, 1.0, 0.0, 0.0);
  const std::vector<WorldPoint> path{{1.5, 1.5}, {2.5, 1.5},
                                     {3.5, 1.5}, {4.5, 1.5}};
  map.grid.set_blocked({8, 8});
  assert(!course_bot_planner::remaining_path_blocked(map, path, {1.5, 1.5}));
  map.grid.set_blocked({3, 1});
  assert(course_bot_planner::remaining_path_blocked(map, path, {1.5, 1.5}));
  map.grid.set_blocked({3, 1}, false);
  map.grid.set_blocked({1, 1});
  assert(!course_bot_planner::remaining_path_blocked(map, path, {4.5, 1.5}));

  // 新障碍只封住斜走的墙角时，端点仍自由，但原 A* 路线已经不合法。
  CourseWorldMap corner(5, 5, 1.0, 0.0, 0.0);
  const std::vector<WorldPoint> diagonal{{1.5, 1.5}, {2.5, 2.5}};
  corner.grid.set_blocked({2, 1});
  assert(course_bot_planner::remaining_path_blocked(corner, diagonal, {1.5, 1.5}));
}
