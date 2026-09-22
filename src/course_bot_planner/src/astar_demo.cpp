#include "course_bot_planner/astar.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using course_bot_planner::Cell;
using course_bot_planner::Grid;

int main(int argc, char **argv) {
  try {
    if (argc != 1 && argc != 5) {
      std::cerr << "Usage: astar_demo [start_x start_y goal_x goal_y]\n";
      return 2;
    }

    // 纯算法演示地图，不等同于 Gazebo 的 course_obstacles.world。
    Grid grid(12, 8);
    for (int y = 0; y <= 5; ++y) grid.set_blocked({5, y});
    for (int x = 7; x <= 9; ++x) grid.set_cost({x, 3}, 5.0);

    Cell start{1, 1};
    Cell goal{10, 6};
    if (argc == 5) {
      start = {std::stoi(argv[1]), std::stoi(argv[2])};
      goal = {std::stoi(argv[3]), std::stoi(argv[4])};
    }

    const auto result = course_bot_planner::astar(grid, start, goal);
    std::vector<std::string> drawing(
        static_cast<std::size_t>(grid.height()),
        std::string(static_cast<std::size_t>(grid.width()), '.'));
    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        const Cell cell{x, y};
        if (!grid.traversable(cell)) drawing[y][x] = '#';
        else if (grid.cost(cell) > 1.0) drawing[y][x] = 'F';
      }
    }
    for (Cell cell : result.path) drawing[cell.y][cell.x] = '*';
    if (grid.in_bounds(start)) drawing[start.y][start.x] = 'S';
    if (grid.in_bounds(goal)) drawing[goal.y][goal.x] = 'G';

    std::cout << "S=start, G=goal, #=wall, F=high cost, *=path\n";
    for (const auto &row : drawing) std::cout << row << '\n';
    std::cout << "Result: " << course_bot_planner::status_name(result.status)
              << ", expanded nodes: " << result.expanded_nodes;
    if (result.status == course_bot_planner::SearchStatus::Found) {
      std::cout << ", path cost: " << std::fixed << std::setprecision(2)
                << result.total_cost << "\nPath:";
      for (Cell cell : result.path) std::cout << " (" << cell.x << ',' << cell.y << ')';
    }
    std::cout << '\n';
    return result.status == course_bot_planner::SearchStatus::Found ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << "astar_demo error: " << error.what() << '\n';
    return 2;
  }
}
