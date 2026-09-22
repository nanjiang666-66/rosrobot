#!/usr/bin/env bash
set -u

PROJECT_ROOT="/mnt/hgfs/ubuntu-workspace/robot_nav_course"
DOCS_DIR="${PROJECT_ROOT}/docs"
REPORT="${DOCS_DIR}/phase1-verification-report.txt"
GAZEBO_LOG="${DOCS_DIR}/gazebo-verification.log"
mkdir -p "${PROJECT_ROOT}/src" "${DOCS_DIR}" "${PROJECT_ROOT}/videos"

talker_pid=""
gazebo_pid=""
failures=0

cleanup() {
  if [[ -n "${talker_pid}" ]] && kill -0 "${talker_pid}" 2>/dev/null; then
    kill -TERM "${talker_pid}" 2>/dev/null || true
    wait "${talker_pid}" 2>/dev/null || true
  fi
  if [[ -n "${gazebo_pid}" ]] && kill -0 "${gazebo_pid}" 2>/dev/null; then
    kill -TERM "${gazebo_pid}" 2>/dev/null || true
    wait "${gazebo_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

exec > >(tee "${REPORT}") 2>&1

pass() { echo "PASS    $*"; }
fail() { echo "FAIL    $*"; failures=$((failures + 1)); }

echo "===== 第一阶段只读验收 ====="
echo "时间：$(date --iso-8601=seconds)"
echo "说明：本脚本不执行 sudo、apt 或软件安装。"
echo

echo "===== 1. 操作系统 ====="
source /etc/os-release
[[ "${VERSION_ID:-}" == "22.04" ]] && pass "Ubuntu ${VERSION_ID}" || fail "需要 Ubuntu 22.04，当前 ${PRETTY_NAME:-未知}"
[[ "$(uname -m)" == "x86_64" ]] && pass "架构 x86_64" || fail "架构 $(uname -m)"
echo "内核：$(uname -r)"
echo

echo "===== 2. ROS 2 环境 ====="
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # ROS/ament setup scripts may probe variables before defining them. Temporarily
  # disable nounset so Bash strict mode does not misclassify that as a failure.
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  pass "/opt/ros/humble/setup.bash"
else
  fail "缺少 /opt/ros/humble/setup.bash"
fi
[[ "${ROS_DISTRO:-}" == "humble" ]] && pass "ROS_DISTRO=humble" || fail "ROS_DISTRO=${ROS_DISTRO:-未设置}"
echo

echo "===== 3. 必需命令 ====="
required_commands=(ros2 gazebo rviz2 colcon rosdep xacro)
for cmd in "${required_commands[@]}"; do
  if command -v "${cmd}" >/dev/null 2>&1; then
    pass "${cmd} -> $(command -v "${cmd}")"
  else
    fail "缺少命令 ${cmd}"
  fi
done
echo

echo "===== 4. 必需 ROS 包 ====="
required_ros_packages=(
  gazebo_ros
  gazebo_plugins
  xacro
  robot_state_publisher
  nav2_bringup
  slam_toolbox
  robot_localization
)
if command -v ros2 >/dev/null 2>&1; then
  for pkg in "${required_ros_packages[@]}"; do
    if prefix="$(ros2 pkg prefix "${pkg}" 2>/dev/null)"; then
      pass "${pkg} -> ${prefix}"
    else
      fail "缺少 ROS 包 ${pkg}"
    fi
  done
else
  fail "ros2 不可用，无法检查 ROS 包"
fi
echo

echo "===== 5. Gazebo版本 ====="
if command -v gazebo >/dev/null 2>&1; then
  gazebo_version_output="$(gazebo --version 2>&1 || true)"
  printf '%s\n' "${gazebo_version_output}"
  if grep -Eq 'version 11([.]|$)' <<<"${gazebo_version_output}"; then
    pass "Gazebo Classic 11"
  else
    fail "未检测到 Gazebo Classic 11"
  fi
else
  fail "Gazebo 不可用"
fi
echo

echo "===== 6. ROS 2发布/订阅测试 ====="
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix demo_nodes_cpp >/dev/null 2>&1; then
  timeout --signal=TERM --kill-after=3 12 \
    ros2 run demo_nodes_cpp talker >"${DOCS_DIR}/talker-verification.log" 2>&1 &
  talker_pid=$!
  sleep 3
  if timeout 15 ros2 topic echo /chatter --once >/dev/null 2>&1; then
    pass "能够从 /chatter 收到消息"
  else
    fail "未能从 /chatter 收到消息"
  fi
  wait "${talker_pid}" 2>/dev/null || true
  talker_pid=""
else
  fail "缺少 ros2 或 demo_nodes_cpp，无法测试"
fi
echo

echo "===== 7. Gazebo与ROS 2联通测试 ====="
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix gazebo_ros >/dev/null 2>&1; then
  : > "${GAZEBO_LOG}"
  if pgrep -x gzserver >/dev/null 2>&1; then
    echo "检测到已有 gzserver，将复用当前实例。"
  else
    timeout --signal=TERM --kill-after=5 45 \
      ros2 launch gazebo_ros gazebo.launch.py gui:=false verbose:=false >"${GAZEBO_LOG}" 2>&1 &
    gazebo_pid=$!
  fi

  clock_ready=0
  for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qx '/clock'; then
      clock_ready=1
      break
    fi
    sleep 1
  done

  if [[ "${clock_ready}" -eq 1 ]] && timeout 15 ros2 topic echo /clock --once >/dev/null 2>&1; then
    pass "Gazebo发布 /clock"
  else
    fail "Gazebo未成功发布 /clock；查看 ${GAZEBO_LOG}"
  fi

  if [[ -n "${gazebo_pid}" ]]; then
    wait "${gazebo_pid}" 2>/dev/null || true
    gazebo_pid=""
  fi
else
  fail "缺少 gazebo_ros，无法测试联通"
fi
echo

echo "===== 最终结果 ====="
if [[ "${failures}" -eq 0 ]]; then
  echo "PHASE_1_RESULT=PASS"
  echo "第一阶段环境验收通过。"
  exit 0
else
  echo "PHASE_1_RESULT=FAIL"
  echo "失败项目数：${failures}"
  echo "只安装报告中明确缺少的组件，不进行全量重装。"
  exit 1
fi
