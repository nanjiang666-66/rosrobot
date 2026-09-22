cmake_minimum_required(VERSION 3.8)
project(course_bot_slam)

# 本包没有自编运行节点，只安装 SLAM Toolbox 的参数、XML 启动和 RViz 配置。
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
