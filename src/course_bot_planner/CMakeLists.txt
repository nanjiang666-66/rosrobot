cmake_minimum_required(VERSION 3.8)
project(course_bot_planner)

# A* 核心不调用 ROS API；ament_cmake 只负责把它纳入现有 colcon 工作空间。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sdformat9 REQUIRED)

add_library(course_bot_astar src/astar.cpp src/course_world_map.cpp src/odom_anchor.cpp)
target_include_directories(course_bot_astar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 单独封装 SDF 依赖，让纯 A* 与栅格地图测试仍保持轻量。
add_library(course_bot_sdf_loader src/sdf_world_loader.cpp)
target_include_directories(course_bot_sdf_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(course_bot_sdf_loader PUBLIC course_bot_astar sdformat9::sdformat9)

add_executable(astar_demo src/astar_demo.cpp)
target_link_libraries(astar_demo course_bot_astar)

add_executable(world_map_demo src/world_map_demo.cpp)
target_link_libraries(world_map_demo course_bot_astar)

add_executable(astar_planner src/astar_planner_node.cpp)
target_link_libraries(astar_planner course_bot_astar course_bot_sdf_loader)
ament_target_dependencies(astar_planner ament_index_cpp rclcpp nav_msgs geometry_msgs gazebo_msgs sensor_msgs tf2_ros)

# 第 7B 的 SLAM 版规划器只消费 /map 与 TF，不发布定位变换。
add_executable(slam_astar_planner src/slam_astar_planner_node.cpp)
target_link_libraries(slam_astar_planner course_bot_astar)
ament_target_dependencies(slam_astar_planner builtin_interfaces rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS course_bot_astar course_bot_sdf_loader astar_demo world_map_demo
  astar_planner slam_astar_planner
  EXPORT export_course_bot_astar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_course_bot_astar HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_astar test/test_astar.cpp)
  target_link_libraries(test_astar course_bot_astar)
  add_test(NAME astar_grid COMMAND test_astar)

  add_executable(test_course_world_map test/test_course_world_map.cpp)
  target_link_libraries(test_course_world_map course_bot_astar)
  add_test(NAME course_world_map COMMAND test_course_world_map)

  add_executable(test_odom_anchor test/test_odom_anchor.cpp)
  target_link_libraries(test_odom_anchor course_bot_astar)
  add_test(NAME odom_anchor COMMAND test_odom_anchor)

  add_executable(test_sdf_world_loader test/test_sdf_world_loader.cpp)
  target_link_libraries(test_sdf_world_loader course_bot_sdf_loader)
  target_compile_definitions(test_sdf_world_loader PRIVATE
    TEST_WORLD_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/world_loader_fixture.world")
  add_test(NAME sdf_world_loader COMMAND test_sdf_world_loader)
endif()

ament_package()
