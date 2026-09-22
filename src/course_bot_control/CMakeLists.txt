cmake_minimum_required(VERSION 3.8)
project(course_bot_control)

# 本包只构建 C++ 节点，不依赖旧 Python 控制包。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(basic_motion src/basic_motion.cpp)
target_include_directories(basic_motion PRIVATE include)
ament_target_dependencies(basic_motion rclcpp geometry_msgs nav_msgs)

add_executable(stop_robot src/stop_robot.cpp)
ament_target_dependencies(stop_robot rclcpp geometry_msgs)

add_executable(path_follower src/path_follower.cpp)
target_include_directories(path_follower PRIVATE include)
ament_target_dependencies(path_follower rclcpp geometry_msgs nav_msgs sensor_msgs)

install(TARGETS basic_motion stop_robot path_follower DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_motion_math test/test_motion_math.cpp)
  target_include_directories(test_motion_math PRIVATE include)
  add_test(NAME motion_math COMMAND test_motion_math)

  add_executable(test_path_following_math test/test_path_following_math.cpp)
  target_include_directories(test_path_following_math PRIVATE include)
  add_test(NAME path_following_math COMMAND test_path_following_math)

  add_executable(test_laser_safety test/test_laser_safety.cpp)
  target_include_directories(test_laser_safety PRIVATE include)
  add_test(NAME laser_safety COMMAND test_laser_safety)
endif()

ament_package()
