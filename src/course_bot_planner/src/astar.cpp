#include "course_bot_planner/astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <queue>
#include <stdexcept>
#include <utility>

namespace course_bot_planner {
namespace {

constexpr double kDiagonalCost = 1.4142135623730951;
constexpr double kEpsilon = 1e-12;

struct OpenEntry {
  double priority;  // f = g + h：优先队列按此值选择下一格。
  double known_cost;  // g：用于识别队列里已经过时的旧记录。
  std::size_t index;
};

struct SmallerPriorityFirst {
  bool operator()(const OpenEntry &a, const OpenEntry &b) const {
    if (a.priority != b.priority) return a.priority > b.priority;
    return a.index > b.index;  // 平局时固定顺序，便于重复测试。
  }
};

double heuristic(Cell a, Cell b, Connectivity connectivity) {
  const double dx = std::abs(static_cast<double>(a.x) - b.x);
  const double dy = std::abs(static_cast<double>(a.y) - b.y);
  if (connectivity == Connectivity::Four) return dx + dy;
  return std::max(dx, dy) + (kDiagonalCost - 1.0) * std::min(dx, dy);
}

}  // namespace

Grid::Grid(int width, int height) : width_(width), height_(height) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("grid width and height must be positive");
  }
  const auto count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  blocked_.assign(count, 0);
  costs_.assign(count, 1.0);
}

bool Grid::in_bounds(Cell cell) const noexcept {
  return cell.x >= 0 && cell.x < width_ && cell.y >= 0 && cell.y < height_;
}

bool Grid::traversable(Cell cell) const noexcept {
  return in_bounds(cell) && blocked_[static_cast<std::size_t>(cell.y) * width_ + cell.x] == 0;
}

std::size_t Grid::index(Cell cell) const {
  if (!in_bounds(cell)) throw std::out_of_range("cell lies outside grid");
  return static_cast<std::size_t>(cell.y) * width_ + cell.x;
}

void Grid::set_blocked(Cell cell, bool blocked) { blocked_[index(cell)] = blocked ? 1 : 0; }

void Grid::set_cost(Cell cell, double cost_value) {
  if (!std::isfinite(cost_value) || cost_value < 1.0) {
    throw std::invalid_argument("traversal cost must be finite and at least 1");
  }
  costs_[index(cell)] = cost_value;
}

double Grid::cost(Cell cell) const { return costs_[index(cell)]; }

SearchResult astar(const Grid &grid, Cell start, Cell goal, Connectivity connectivity) {
  SearchResult result;
  if (!grid.traversable(start)) {
    result.status = SearchStatus::InvalidStart;
    return result;
  }
  if (!grid.traversable(goal)) {
    result.status = SearchStatus::InvalidGoal;
    return result;
  }

  const auto width = static_cast<std::size_t>(grid.width());
  const auto count = width * static_cast<std::size_t>(grid.height());
  const auto to_index = [width](Cell cell) {
    return static_cast<std::size_t>(cell.y) * width + static_cast<std::size_t>(cell.x);
  };
  const auto to_cell = [width](std::size_t index) {
    return Cell{static_cast<int>(index % width), static_cast<int>(index / width)};
  };
  const std::size_t start_index = to_index(start);
  const std::size_t goal_index = to_index(goal);

  std::vector<double> g_score(count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> came_from(count, count);  // count 表示“尚无前驱”。
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, SmallerPriorityFirst> frontier;
  g_score[start_index] = 0.0;
  came_from[start_index] = start_index;
  frontier.push({heuristic(start, goal, connectivity), 0.0, start_index});

  constexpr std::array<Cell, 8> directions{{
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
  }};
  const int direction_count = connectivity == Connectivity::Four ? 4 : 8;

  while (!frontier.empty()) {
    const OpenEntry entry = frontier.top();
    frontier.pop();
    if (entry.known_cost > g_score[entry.index] + kEpsilon) continue;
    ++result.expanded_nodes;

    if (entry.index == goal_index) {
      result.status = SearchStatus::Found;
      result.total_cost = g_score[goal_index];
      // 前驱从终点指向起点；最后反转成机器人行驶方向。
      for (std::size_t at = goal_index;; at = came_from[at]) {
        result.path.push_back(to_cell(at));
        if (at == start_index) break;
      }
      std::reverse(result.path.begin(), result.path.end());
      return result;
    }

    const Cell current = to_cell(entry.index);
    for (int i = 0; i < direction_count; ++i) {
      const Cell delta = directions[static_cast<std::size_t>(i)];
      const Cell next{current.x + delta.x, current.y + delta.y};
      if (!grid.traversable(next)) continue;
      const bool diagonal = delta.x != 0 && delta.y != 0;
      if (diagonal &&
          (!grid.traversable({current.x + delta.x, current.y}) ||
           !grid.traversable({current.x, current.y + delta.y}))) {
        continue;  // 两侧有墙时不能从墙角斜穿过去。
      }
      const std::size_t next_index = to_index(next);
      const double step_cost = (diagonal ? kDiagonalCost : 1.0) * grid.cost(next);
      const double new_cost = g_score[entry.index] + step_cost;
      if (new_cost + kEpsilon < g_score[next_index]) {
        g_score[next_index] = new_cost;
        came_from[next_index] = entry.index;
        frontier.push({new_cost + heuristic(next, goal, connectivity), new_cost, next_index});
      }
    }
  }

  result.status = SearchStatus::NoPath;
  return result;
}

const char *status_name(SearchStatus status) noexcept {
  switch (status) {
    case SearchStatus::Found: return "found";
    case SearchStatus::InvalidStart: return "invalid start";
    case SearchStatus::InvalidGoal: return "invalid goal";
    case SearchStatus::NoPath: return "no path";
  }
  return "unknown";
}

}  // namespace course_bot_planner
