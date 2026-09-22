#include "course_bot_planner/course_world_map.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>


using course_bot_planner::Cell;
using course_bot_planner::CourseWorldMap;
using course_bot_planner::WorldPoint;

int main(int argc, char **argv) {
  try {
    if (argc != 1 && argc != 5) {
      std::cerr << "Usage: world_map_demo [start_x_m start_y_m goal_x_m goal_y_m]\n";
      return 2;
    }
    WorldPoint start_world = course_bot_planner::kDefaultStartWorld;
    WorldPoint goal_world = course_bot_planner::kDefaultGoalWorld;
    if (argc == 5) {
      start_world = {std::stod(argv[1]), std::stod(argv[2])};
      goal_world = {std::stod(argv[3]), std::stod(argv[4])};
    }

    CourseWorldMap map;
    const auto start = map.world_to_cell(start_world);
    const auto goal = map.world_to_cell(goal_world);
    if (!start || !goal) {
      std::cerr << "Start or goal lies outside the 10 m x 10 m map.\n";
      return 2;
    }
    const auto result = course_bot_planner::astar(
        map.grid, *start, *goal, course_bot_planner::Connectivity::Eight);

    std::vector<std::string> drawing(
        CourseWorldMap::kHeight, std::string(CourseWorldMap::kWidth, '.'));
    for (int y = 0; y < CourseWorldMap::kHeight; ++y) {
      for (int x = 0; x < CourseWorldMap::kWidth; ++x) {
        if (!map.grid.traversable({x, y})) drawing[y][x] = '#';
      }
    }
    for (Cell cell : result.path) drawing[cell.y][cell.x] = '*';
    drawing[start->y][start->x] = 'S';
    drawing[goal->y][goal->x] = 'G';

    std::cout << "Gazebo-aligned static grid: origin (-5,-5) m, 0.20 m/cell, "
              << "50 x 50 cells\n";
    std::cout << "S=start, G=goal, #=inflated obstacle/boundary, *=A* path\n";
    std::cout << "Rows are printed from positive y (top) to negative y (bottom).\n";
    for (int y = CourseWorldMap::kHeight - 1; y >= 0; --y) {
      std::cout << drawing[y] << '\n';
    }
    std::cout << std::fixed << std::setprecision(2)
              << "Start (m): (" << start_world.x << ", " << start_world.y << ")"
              << " -> cell (" << start->x << ", " << start->y << ")\n"
              << "Goal  (m): (" << goal_world.x << ", " << goal_world.y << ")"
              << " -> cell (" << goal->x << ", " << goal->y << ")\n"
              << "Result: " << course_bot_planner::status_name(result.status)
              << ", expanded nodes: " << result.expanded_nodes;
    if (result.status == course_bot_planner::SearchStatus::Found) {
      std::cout << ", path cost: " << result.total_cost
                << ", waypoints: " << result.path.size();
    }
    std::cout << '\n';
    return result.status == course_bot_planner::SearchStatus::Found ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << "world_map_demo error: " << error.what() << '\n';
    return 2;
  }
}
