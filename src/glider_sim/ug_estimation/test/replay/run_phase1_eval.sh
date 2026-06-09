#!/usr/bin/env bash
# Phase 1 离线评估：用 baseline bag 同时回放跑 ug_eskf + robot_localization，
# 录制两者输出 + 真值 + diagnostics，再用 analyze.py 出 RMSE/NIS 报告。
#
# 用法:
#   bash run_phase1_eval.sh <input_baseline.bag> [namespace] [init_yaw_ned_rad]
#
# 注意:
#   - 仅本地评估用，不进 CI。
#   - 输入 bag 必须含 /<ns>/imu /<ns>/pressure /<ns>/ground_truth/pose /clock。
#   - 用 sim time 回放，确保 EKF 用 bag 内时间戳算 dt。

set -uo pipefail

BAG_IN="${1:?用法: run_phase1_eval.sh <input.bag> [ns] [init_yaw_ned_rad]}"
NS="${2:-ug_glider}"
INIT_YAW="${3:-0.0}"

if [[ ! -f "${BAG_IN}" ]]; then
  echo "[eval] 找不到输入 bag: ${BAG_IN}"; exit 1
fi

WS_ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
source "${WS_ROOT}/devel/setup.bash"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$(dirname "${BAG_IN}")"
BAG_OUT="${OUT_DIR}/eval_out_${STAMP}.bag"

PIDS=()
cleanup() {
  echo "[eval] 清理进程..."
  # record 用 SIGINT 干净收尾（让 .bag 正常重命名，不留 .active）
  pkill -INT -f "rosbag record -O ${BAG_OUT}" 2>/dev/null || true
  sleep 2
  for p in "${PIDS[@]}"; do kill "${p}" 2>/dev/null || true; done
  pkill -f "rosbag play" 2>/dev/null || true
  pkill -f "estimation.launch" 2>/dev/null || true
  kill "${ROSCORE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

# ★ 关键加固：启动前强清理任何 ROS 残留，避免上一次 run 的 rosbag play/record
#   叠加进来污染本次时间戳（曾导致 eval_out 时间戳回环、pitch 评估失真）。
echo "[eval] 预清理残留 ROS 进程..."
pkill -9 -f "rosbag play"   2>/dev/null || true
pkill -9 -f "rosbag record" 2>/dev/null || true
pkill -9 -f "glider_ekf"    2>/dev/null || true
pkill -9 -f "ekf_localization" 2>/dev/null || true
pkill -9 -f "estimation.launch" 2>/dev/null || true
pkill -9 -f "rosmaster"     2>/dev/null || true
pkill -9 -f "roscore"       2>/dev/null || true
sleep 3

echo "[eval] 启动 roscore"
roscore >/dev/null 2>&1 &
ROSCORE_PID=$!
sleep 5
rosparam set /use_sim_time true

echo "[eval] 启动两个估计器 (ug_eskf + robot_localization)"
roslaunch ug_estimation estimation.launch namespace:="${NS}" \
    estimator:=ug_eskf init_yaw_ned_rad:="${INIT_YAW}" >/dev/null 2>&1 &
PIDS+=($!)
roslaunch ug_estimation estimation.launch namespace:="${NS}" \
    estimator:=robot_localization >/dev/null 2>&1 &
PIDS+=($!)
sleep 5
# 确认只有一个 EKF 节点（防叠加）
NODECNT=$(rosnode list 2>/dev/null | grep -c glider_ekf || true)
echo "[eval] glider_ekf 节点数=${NODECNT} (应为 1)"

REC_TOPICS=(
  "/${NS}/ground_truth/pose"
  "/${NS}/odometry/filtered_eskf"
  "/${NS}/odometry/filtered"
  "/${NS}/glider_ekf/diagnostics"
)
echo "[eval] 录制输出到 ${BAG_OUT}"
rosbag record -O "${BAG_OUT}" "${REC_TOPICS[@]}" >/dev/null 2>&1 &
PIDS+=($!)
sleep 2

echo "[eval] 回放 ${BAG_IN} (--clock)"
rosbag play --clock "${BAG_IN}"

echo "[eval] 回放结束，等待缓冲落盘"
sleep 3
# SIGINT 让 record 干净收尾并把 .active 重命名为 .bag
pkill -INT -f "rosbag record -O ${BAG_OUT}" 2>/dev/null || true
sleep 3

# 若仍是 .active（未干净收尾），reindex 抢救
if [[ ! -f "${BAG_OUT}" && -f "${BAG_OUT}.active" ]]; then
  echo "[eval] 检测到 .active，reindex 抢救"
  mv "${BAG_OUT}.active" "${BAG_OUT}"
  rosbag reindex "${BAG_OUT}" >/dev/null 2>&1 || true
fi

echo "[eval] 分析"
python3 "$(dirname "$0")/analyze.py" "${BAG_OUT}" --ns "${NS}"

echo "[eval] 完成。输出 bag: ${BAG_OUT}"
