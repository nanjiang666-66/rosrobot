#include "course_bot_planner/astar.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

using course_bot_planner::Cell;
using course_bot_planner::Connectivity;
using course_bot_planner::Grid;
using course_bot_planner::SearchResult;
using course_bot_planner::SearchStatus;

namespace {

void expect(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

void expect_near(double actual, double expected, const std::string &message) {
  expect(std::abs(actual - expected) < 1e-9, message);
}

void check_path(const Grid &grid, const SearchResult &result, Cell start, Cell goal,
                Connectivity connectivity = Connectivity::Four) {
  expect(result.status == SearchStatus::Found, "expected a path");
  expect(!result.path.empty(), "found path is empty");
  expect(result.path.front() == start, "path does not start at start");
  expect(result.path.back() == goal, "path does not end at goal");
  double measured_cost = 0.0;
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    const Cell cell = result.path[i];
    expect(grid.traversable(cell), "path enters a wall");
    if (i == 0) continue;
    const Cell previous = result.path[i - 1];
    const int dx = std::abs(cell.x - previous.x);
    const int dy = std::abs(cell.y - previous.y);
    expect(dx <= 1 && dy <= 1 && dx + dy >= 1, "path has a non-neighbor jump");
    const bool diagonal = dx == 1 && dy == 1;
    expect(connectivity == Connectivity::Eight || !diagonal,
           "four-neighbor path contains a diagonal step");
    if (diagonal) {
      expect(grid.traversable({cell.x, previous.y}) &&
                 grid.traversable({previous.x, cell.y}),
             "diagonal path cuts through a wall corner");
    }
    measured_cost += (diagonal ? std::sqrt(2.0) : 1.0) * grid.cost(cell);
  }
  expect_near(measured_cost, result.total_cost, "reported path cost is incorrect");
}

void test_open_grid() {
  Grid grid(5, 5);
  const auto result = course_bot_planner::astar(grid, {0, 0}, {4, 4});
  check_path(grid, result, {0, 0}, {4, 4});
  expect_near(result.total_cost, 8.0, "open-grid route should cost 8");
  expect(result.path.size() == 9, "open-grid route should contain 9 cells");
}

void test_wall_detour() {
  Grid grid(7, 5);
  for (int y = 0; y < 4; ++y) grid.set_blocked({3, y});
  const auto result = course_bot_planner::astar(grid, {1, 1}, {5, 1});
  check_path(grid, result, {1, 1}, {5, 1});
  expect_near(result.total_cost, 10.0, "wall detour should cost 10");
}

void test_narrow_passage() {
  Grid grid(5, 5);
  for (int y = 0; y < 5; ++y) {
    if (y != 2) grid.set_blocked({2, y});
  }
  const auto result = course_bot_planner::astar(grid, {0, 2}, {4, 2});
  check_path(grid, result, {0, 2}, {4, 2});
  expect_near(result.total_cost, 4.0, "narrow passage should cost 4");
}

void test_weighted_detour() {
  Grid grid(5, 3);
  for (int x = 1; x <= 3; ++x) grid.set_cost({x, 1}, 5.0);
  const auto result = course_bot_planner::astar(grid, {0, 1}, {4, 1});
  check_path(grid, result, {0, 1}, {4, 1});
  expect_near(result.total_cost, 6.0, "weighted route should avoid expensive cells");
  for (Cell cell : result.path) {
    expect(grid.cost(cell) == 1.0, "weighted path enters a high-cost cell");
  }
}

void test_no_path() {
  Grid grid(5, 5);
  for (int y = 0; y < 5; ++y) grid.set_blocked({2, y});
  const auto result = course_bot_planner::astar(grid, {0, 2}, {4, 2});
  expect(result.status == SearchStatus::NoPath, "wall should make goal unreachable");
  expect(result.path.empty(), "no-path result should not contain a path");
  expect(std::isinf(result.total_cost), "no-path cost should be infinity");
}

void test_invalid_endpoints() {
  Grid grid(3, 3);
  grid.set_blocked({0, 0});
  grid.set_blocked({2, 2});
  expect(course_bot_planner::astar(grid, {0, 0}, {1, 1}).status ==
             SearchStatus::InvalidStart,
         "blocked start should be rejected");
  expect(course_bot_planner::astar(grid, {1, 1}, {2, 2}).status ==
             SearchStatus::InvalidGoal,
         "blocked goal should be rejected");
  expect(course_bot_planner::astar(grid, {-1, 0}, {1, 1}).status ==
             SearchStatus::InvalidStart,
         "out-of-bounds start should be rejected");
  expect(course_bot_planner::astar(grid, {1, 1}, {3, 1}).status ==
             SearchStatus::InvalidGoal,
         "out-of-bounds goal should be rejected");
}

void test_same_start_goal() {
  Grid grid(3, 3);
  const auto result = course_bot_planner::astar(grid, {1, 1}, {1, 1});
  check_path(grid, result, {1, 1}, {1, 1});
  expect(result.path.size() == 1, "same-start-goal path should contain one cell");
  expect_near(result.total_cost, 0.0, "same-start-goal cost should be zero");
}

void test_eight_neighbors_and_corner_cutting() {
  Grid open_grid(3, 3);
  const auto direct = course_bot_planner::astar(open_grid, {0, 0}, {2, 2},
                                                Connectivity::Eight);
  check_path(open_grid, direct, {0, 0}, {2, 2}, Connectivity::Eight);
  expect_near(direct.total_cost, 2.0 * std::sqrt(2.0),
              "two diagonal steps should cost 2*sqrt(2)");

  Grid blocked_grid(2, 2);
  blocked_grid.set_blocked({1, 0});
  blocked_grid.set_blocked({0, 1});
  const auto blocked = course_bot_planner::astar(blocked_grid, {0, 0}, {1, 1},
                                                 Connectivity::Eight);
  expect(blocked.status == SearchStatus::NoPath,
         "diagonal movement must not cut through a wall corner");
}

void test_bad_cost() {
  Grid grid(2, 2);
  bool rejected = false;
  try {
    grid.set_cost({0, 0}, 0.5);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  expect(rejected, "cost below 1 should be rejected to protect heuristic bound");
}

}  // namespace

int main() {
  try {
    test_open_grid();
    test_wall_detour();
    test_narrow_passage();
    test_weighted_detour();
    test_no_path();
    test_invalid_endpoints();
    test_same_start_goal();
    test_eight_neighbors_and_corner_cutting();
    test_bad_cost();
    std::cout << "All C++ A* tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "A* test failed: " << error.what() << '\n';
    return 1;
  }
}
