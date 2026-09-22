#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 原构建目录含 ament_python 产物；改用新的本机目录，避免切换到 CMake 后混用缓存。
output_dir="${HOME}/robot_nav_course_colcon_cpp"

source /opt/ros/humble/setup.bash
mkdir -p "${output_dir}/build" "${output_dir}/install" "${output_dir}/log"

colcon --log-base "${output_dir}/log" build \
  --base-paths "${workspace_dir}/src" \
  --build-base "${output_dir}/build" \
  --install-base "${output_dir}/install" \
  --event-handlers console_direct+

echo
echo "Build completed. Run:"
echo "  source ${output_dir}/install/setup.bash"
echo "  ros2 launch course_bot_slam navigation.launch.xml"
