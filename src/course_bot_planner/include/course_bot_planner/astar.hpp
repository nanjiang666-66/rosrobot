#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace course_bot_planner {

// 格子坐标，不是 Gazebo 世界中的米制坐标；坐标转换将在接入 ROS 时完成。
struct Cell {
  int x{};
  int y{};
};

inline bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Cell a, Cell b) { return !(a == b); }

enum class Connectivity { Four, Eight };
enum class SearchStatus { Found, InvalidStart, InvalidGoal, NoPath };

class Grid {
public:
  Grid(int width, int height);

  int width() const noexcept { return width_; }
  int height() const noexcept { return height_; }
  bool in_bounds(Cell cell) const noexcept;
  bool traversable(Cell cell) const noexcept;

  // 障碍格不可通行；普通格进入代价为 1，高代价格可以模拟森林等区域。
  void set_blocked(Cell cell, bool blocked = true);
  void set_cost(Cell cell, double cost);
  double cost(Cell cell) const;

private:
  std::size_t index(Cell cell) const;
  int width_;
  int height_;
  std::vector<std::uint8_t> blocked_;
  std::vector<double> costs_;
};

struct SearchResult {
  SearchStatus status{SearchStatus::NoPath};
  std::vector<Cell> path;  // 成功时包含起点和终点；失败时为空。
  double total_cost{std::numeric_limits<double>::infinity()};
  std::size_t expanded_nodes{0};
};

// 4 邻域用曼哈顿距离；8 邻域用八方向距离，并禁止穿过障碍物的斜角。
// 所有格子的进入代价必须 >= 1，保证启发式不会高估真实剩余代价。
SearchResult astar(const Grid &grid, Cell start, Cell goal,
                   Connectivity connectivity = Connectivity::Four);

const char *status_name(SearchStatus status) noexcept;

}  // namespace course_bot_planner
