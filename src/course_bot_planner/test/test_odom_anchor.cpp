#include "course_bot_planner/odom_anchor.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

using course_bot_planner::OdomAnchor;
using course_bot_planner::Pose2D;

namespace {
void expect_near(double actual, double expected, const std::string &label) {
  if (std::abs(actual - expected) > 1e-9) {
    throw std::runtime_error(label + " differs from expected value");
  }
}

void test_zero_origin_odom() {
  const OdomAnchor anchor({0.0, 0.0, 0.0}, {-4.0, -3.0, 0.0});
  const Pose2D current = anchor.odom_pose_in_map({1.0, 2.0, 0.5});
  expect_near(current.x, -3.0, "zero-origin x");
  expect_near(current.y, -1.0, "zero-origin y");
  expect_near(current.yaw, 0.5, "zero-origin yaw");
}

void test_world_aligned_odom() {
  const OdomAnchor anchor({-4.0, -3.0, 0.0}, {-4.0, -3.0, 0.0});
  const Pose2D current = anchor.odom_pose_in_map({-3.0, -1.0, 0.5});
  expect_near(current.x, -3.0, "world-aligned x");
  expect_near(current.y, -1.0, "world-aligned y");
  expect_near(current.yaw, 0.5, "world-aligned yaw");
}

void test_rotated_odom_frame() {
  const double half_pi = std::acos(-1.0) / 2.0;
  const OdomAnchor anchor({2.0, 1.0, half_pi}, {-4.0, -3.0, 0.0});
  const Pose2D first = anchor.odom_pose_in_map({2.0, 1.0, half_pi});
  expect_near(first.x, -4.0, "rotated initial x");
  expect_near(first.y, -3.0, "rotated initial y");
  expect_near(first.yaw, 0.0, "rotated initial yaw");
  const Pose2D moved = anchor.odom_pose_in_map({2.0, 2.0, half_pi});
  expect_near(moved.x, -3.0, "rotated moved x");
  expect_near(moved.y, -3.0, "rotated moved y");
}
}  // namespace

int main() {
  try {
    test_zero_origin_odom();
    test_world_aligned_odom();
    test_rotated_odom_frame();
    std::cout << "Odom anchor C++ tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Odom anchor test failed: " << error.what() << '\n';
    return 1;
  }
}
