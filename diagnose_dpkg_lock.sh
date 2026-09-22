#!/usr/bin/env bash
set -u

locks=(
  /var/lib/dpkg/lock-frontend
  /var/lib/dpkg/lock
  /var/lib/apt/lists/lock
  /var/cache/apt/archives/lock
)

echo "===== 时间 ====="
date --iso-8601=seconds

echo
echo "===== 当前锁占用 ====="
sudo fuser -v "${locks[@]}" 2>&1 || true

pids="$(sudo fuser "${locks[@]}" 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -nu)"

echo
echo "===== 占锁进程详情 ====="
if [[ -z "${pids}" ]]; then
  echo "当前没有进程占用 apt/dpkg 锁。"
else
  for pid in ${pids}; do
    ps -p "${pid}" -o pid,ppid,user,stat,etime,%cpu,%mem,lstart,cmd --no-headers || true
    if command -v pstree >/dev/null 2>&1; then
      pstree -sp "${pid}" || true
    fi
  done
fi

echo
echo "===== apt 自动更新服务 ====="
systemctl --no-pager --full status apt-daily.service apt-daily-upgrade.service unattended-upgrades.service 2>&1 \
  | tail -n 80 || true

echo
echo "===== 最近 apt 日志 ====="
sudo tail -n 40 /var/log/apt/term.log 2>&1 || true

echo
echo "===== 最近 unattended-upgrades 日志 ====="
sudo tail -n 40 /var/log/unattended-upgrades/unattended-upgrades.log 2>&1 || true

echo
echo "===== 诊断结束 ====="
echo "本脚本只读取状态，没有停止进程，也没有删除锁文件。"
