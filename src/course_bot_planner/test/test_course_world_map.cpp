#include "course_bot_planner/course_world_map.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

using course_bot_planner::Cell;
using course_bot_planner::CourseWorldMap;
using course_bot_planner::SearchStatus;
using course_bot_planner::WorldPoint;

namespace {

void expect(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

Cell require_cell(const CourseWorldMap &map, WorldPoint point) {
  const auto cell = map.world_to_cell(point);
  expect(cell.has_value(), "expected point inside map");
  return *cell;
}

void test_geometry_and_coordinates() {
  const CourseWorldMap map;
  expect(map.grid.width() == 50 && map.grid.height() == 50,
         "unexpected map dimensions");
  expect(!map.world_to_cell({5.0, 0.0}).has_value(), "right edge must be excluded");
  expect(!map.world_to_cell({0.0, -5.1}).has_value(), "outside point must be rejected");
  expect(!map.world_to_cell({INFINITY, 0.0}).has_value(), "infinite point must be rejected");

  // 四处实物障碍的中心均来自 course_obstacles.world。
  expect(!map.grid.traversable(require_cell(map, {-1.0, -1.2})), "box 1 missing");
  expect(!map.grid.traversable(require_cell(map, {1.5, 1.0})), "box 2 missing");
  expect(!map.grid.traversable(require_cell(map, {3.0, -1.5})), "cylinder missing");
  expect(!map.grid.traversable(require_cell(map, {0.5, -2.7})), "sphere missing");
  expect(!map.grid.traversable({0, 0}), "planning boundary should be blocked");

  // Gazebo 生成点和预选终点应在膨胀后的自由区。
  const Cell start = require_cell(map, course_bot_planner::kDefaultStartWorld);
  const Cell goal = require_cell(map, course_bot_planner::kDefaultGoalWorld);
  expect(map.grid.traversable(start), "Gazebo spawn position is occupied");
  expect(map.grid.traversable(goal), "proposed goal is occupied");
  const WorldPoint center = map.cell_center(start);
  expect(require_cell(map, center) == start, "world/cell conversion does not round-trip");
  expect(std::abs(center.x - course_bot_planner::kDefaultStartWorld.x) <=
             CourseWorldMap::kResolution / 2.0 + 1e-12,
         "start conversion error exceeds half a cell");
}

void test_start_to_goal_route() {
  const CourseWorldMap map;
  const Cell start = require_cell(map, course_bot_planner::kDefaultStartWorld);
  const Cell goal = require_cell(map, course_bot_planner::kDefaultGoalWorld);
  const auto result = course_bot_planner::astar(
      map.grid, start, goal, course_bot_planner::Connectivity::Eight);
  expect(result.status == SearchStatus::Found, "default Gazebo route not found");
  expect(result.path.front() == start && result.path.back() == goal,
         "default path endpoints are wrong");
  expect(result.path.size() > 2, "route should contain intermediate waypoints");
  for (Cell cell : result.path) {
    expect(map.grid.traversable(cell), "route crosses inflated obstacle");
  }
  expect(std::isfinite(result.total_cost), "default route cost must be finite");
}

void test_runtime_map_geometry() {
  const CourseWorldMap map(8, 6, 0.5, -2.0, -1.0);
  expect(map.grid.width() == 8 && map.grid.height() == 6,
         "runtime map dimensions are wrong");
  expect(std::abs(map.resolution() - 0.5) < 1e-12,
         "runtime map resolution is wrong");
  const auto cell = map.world_to_cell({-0.75, 0.25});
  expect(cell.has_value() && *cell == Cell{2, 2},
         "runtime world_to_cell conversion is wrong");
  const auto center = map.cell_center(*cell);
  expect(std::abs(center.x + 0.75) < 1e-12 &&
             std::abs(center.y - 0.25) < 1e-12,
         "runtime cell_center conversion is wrong");
  expect(!map.world_to_cell({2.0, 0.0}).has_value(),
         "runtime map right edge must be excluded");
}

}  // namespace

int main() {
  try {
    test_geometry_and_coordinates();
    test_start_to_goal_route();
    test_runtime_map_geometry();
    std::cout << "Course world map C++ tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Course world map test failed: " << error.what() << '\n';
    return 1;
  }
}
