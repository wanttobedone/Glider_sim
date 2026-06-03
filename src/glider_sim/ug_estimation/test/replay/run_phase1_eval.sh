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
  for p in "${PIDS[@]}"; do kill "${p}" 2>/dev/null || true; done
  pkill -f "rosbag record -O ${BAG_OUT}" 2>/dev/null || true
  kill "${ROSCORE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

echo "[eval] 启动 roscore"
roscore >/dev/null 2>&1 &
ROSCORE_PID=$!
sleep 3
rosparam set /use_sim_time true

echo "[eval] 启动两个估计器 (ug_eskf + robot_localization)"
roslaunch ug_estimation estimation.launch namespace:="${NS}" \
    estimator:=ug_eskf init_yaw_ned_rad:="${INIT_YAW}" >/dev/null 2>&1 &
PIDS+=($!)
roslaunch ug_estimation estimation.launch namespace:="${NS}" \
    estimator:=robot_localization >/dev/null 2>&1 &
PIDS+=($!)
sleep 4

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
pkill -f "rosbag record -O ${BAG_OUT}" 2>/dev/null || true
sleep 2

echo "[eval] 分析"
python3 "$(dirname "$0")/analyze.py" "${BAG_OUT}" --ns "${NS}"

echo "[eval] 完成。输出 bag: ${BAG_OUT}"
