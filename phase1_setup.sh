#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="/mnt/hgfs/ubuntu-workspace/robot_nav_course"
DOCS_DIR="${PROJECT_ROOT}/docs"
REPORT="${DOCS_DIR}/environment-report.txt"
TALKER_LOG="${DOCS_DIR}/talker-test.log"
GAZEBO_LOG="${DOCS_DIR}/gazebo-headless-test.log"
CLOCK_LOG="${DOCS_DIR}/clock-sample.txt"

mkdir -p "${PROJECT_ROOT}/src" "${DOCS_DIR}" "${PROJECT_ROOT}/videos"

talker_pid=""
gazebo_pid=""
cleanup() {
  if [[ -n "${talker_pid}" ]] && kill -0 "${talker_pid}" 2>/dev/null; then
    kill -INT "${talker_pid}" 2>/dev/null || true
    wait "${talker_pid}" 2>/dev/null || true
  fi
  if [[ -n "${gazebo_pid}" ]] && kill -0 "${gazebo_pid}" 2>/dev/null; then
    kill -INT "${gazebo_pid}" 2>/dev/null || true
    wait "${gazebo_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[1/8] 验证操作系统"
source /etc/os-release
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  echo "错误：需要 Ubuntu 22.04，当前为 ${PRETTY_NAME:-未知系统}" >&2
  exit 1
fi
if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "错误：需要 x86_64，当前为 $(uname -m)" >&2
  exit 1
fi

echo "[2/8] 获取 sudo 授权并补齐软件包"
sudo -v

wait_for_dpkg_lock() {
  local attempt holder_pids pid
  echo "检查 apt/dpkg 锁；若系统正在自动更新，将最多等待 90 分钟……"
  for attempt in $(seq 1 1080); do
    if ! sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 \
      && ! sudo fuser /var/lib/dpkg/lock >/dev/null 2>&1; then
      echo "apt/dpkg 锁已释放。"
      return 0
    fi
    if (( attempt == 1 || attempt % 12 == 0 )); then
      echo "仍有软件包管理进程运行，继续等待（已等待 $((attempt * 5)) 秒）……"
      sudo fuser -v /var/lib/dpkg/lock-frontend /var/lib/dpkg/lock 2>/dev/null || true
      holder_pids="$(sudo fuser /var/lib/dpkg/lock-frontend /var/lib/dpkg/lock 2>/dev/null \
        | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -nu || true)"
      for pid in ${holder_pids}; do
        ps -p "${pid}" -o pid,stat,etime,%cpu,%mem,cmd --no-headers || true
      done
      echo "最近的软件包安装进度："
      sudo tail -n 6 /var/log/apt/term.log 2>/dev/null || true
    fi
    sleep 5
  done
  echo "错误：等待 90 分钟后 apt/dpkg 仍被占用。不要删除锁文件，请先检查占锁进程。" >&2
  sudo fuser -v /var/lib/dpkg/lock-frontend /var/lib/dpkg/lock 2>/dev/null || true
  return 1
}

wait_for_dpkg_lock
sudo dpkg --configure -a
sudo apt-get -o DPkg::Lock::Timeout=600 update
sudo apt-get -o DPkg::Lock::Timeout=600 install -y \
  ros-humble-desktop \
  ros-dev-tools \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools \
  ros-humble-teleop-twist-keyboard \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  python3-colcon-common-extensions \
  python3-rosdep

echo "[3/8] 配置 ROS 2 Humble 环境"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "错误：/opt/ros/humble/setup.bash 不存在" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
grep -qxF 'source /opt/ros/humble/setup.bash' "${HOME}/.bashrc" \
  || echo 'source /opt/ros/humble/setup.bash' >> "${HOME}/.bashrc"

echo "[4/8] 初始化 rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "[5/8] 检查命令与软件包"
required_commands=(ros2 gazebo rviz2 colcon rosdep xacro)
for cmd in "${required_commands[@]}"; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "错误：缺少命令 ${cmd}" >&2
    exit 1
  fi
done

required_ros_packages=(
  gazebo_ros
  gazebo_plugins
  xacro
  robot_state_publisher
  nav2_bringup
  slam_toolbox
  robot_localization
)
for pkg in "${required_ros_packages[@]}"; do
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "错误：缺少 ROS 包 ${pkg}" >&2
    exit 1
  fi
done

echo "[6/8] 自动测试 ROS 2 发布/订阅"
: > "${TALKER_LOG}"
ros2 run demo_nodes_cpp talker >"${TALKER_LOG}" 2>&1 &
talker_pid=$!
sleep 3
if ! timeout 15 ros2 topic echo /chatter --once >/dev/null 2>&1; then
  echo "错误：未能从 /chatter 收到消息" >&2
  exit 1
fi
kill -INT "${talker_pid}" 2>/dev/null || true
wait "${talker_pid}" 2>/dev/null || true
talker_pid=""

echo "[7/8] 无界面测试 Gazebo 与 ROS 2 /clock"
: > "${GAZEBO_LOG}"
: > "${CLOCK_LOG}"
if pgrep -x gzserver >/dev/null 2>&1; then
  echo "检测到已有 gzserver，复用当前 Gazebo 实例" >>"${GAZEBO_LOG}"
else
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
if [[ "${clock_ready}" -ne 1 ]]; then
  echo "错误：30秒内未发现 /clock，请查看 ${GAZEBO_LOG}" >&2
  exit 1
fi
if ! timeout 15 ros2 topic echo /clock --once >"${CLOCK_LOG}" 2>&1; then
  echo "错误：/clock 存在但没有收到数据，请查看 ${CLOCK_LOG}" >&2
  exit 1
fi
if [[ -n "${gazebo_pid}" ]]; then
  kill -INT "${gazebo_pid}" 2>/dev/null || true
  wait "${gazebo_pid}" 2>/dev/null || true
  gazebo_pid=""
fi

echo "[8/8] 生成环境验收报告"
{
  echo "===== PHASE 1 RESULT: PASS ====="
  echo "Date: $(date --iso-8601=seconds)"
  echo
  echo "===== UBUNTU ====="
  echo "PRETTY_NAME=${PRETTY_NAME}"
  echo "VERSION_CODENAME=${VERSION_CODENAME:-}"
  echo "ARCH=$(uname -m)"
  echo "KERNEL=$(uname -r)"
  echo
  echo "===== ROS 2 ====="
  echo "ROS_DISTRO=${ROS_DISTRO:-}"
  echo "ROS2=$(command -v ros2)"
  echo
  echo "===== GAZEBO ====="
  gazebo --version
  echo
  echo "===== TOOLS ====="
  echo "RVIZ2=$(command -v rviz2)"
  echo "COLCON=$(command -v colcon)"
  echo "ROSDEP=$(command -v rosdep)"
  echo "XACRO=$(command -v xacro)"
  echo
  echo "===== REQUIRED ROS PACKAGES ====="
  for pkg in "${required_ros_packages[@]}"; do
    echo "${pkg}: $(ros2 pkg prefix "${pkg}")"
  done
  echo
  echo "===== AUTOMATED TESTS ====="
  echo "ROS2_PUB_SUB=PASS"
  echo "GAZEBO_ROS_CLOCK=PASS"
  echo "RVIZ2_EXECUTABLE=PASS"
} | tee "${REPORT}"

echo
echo "第一阶段自动配置与测试已完成。"
echo "报告位置：${REPORT}"
echo "Windows位置：D:\\Projects\\ubuntu-workspace\\robot_nav_course\\docs\\environment-report.txt"
echo "说明：RViz2图形窗口仍需后续人工打开一次确认显示效果。"
