#!/usr/bin/env bash
# 录制 EKF 验证基线 rosbag。
#
# 流程：
#   1. 假设 full_sim.launch 已经在另一个终端启动（estimation:=false 或 true 都行，
#      因为 baseline 录的是传感器原始数据，不是 EKF 输出）。
#   2. 本脚本启动任务节点 (mission_node) 并 rosbag record 一段时间，存到 data/。
#
# 用法：
#   bash record_baseline.sh [duration_sec]
#
# 标准参数（baseline_spec.md 同步维护）：
#   namespace : ug_glider
#   任务      : zigzag, dive_depth=30, climb_depth=2
#   时长      : 默认 600 s (10 min)，覆盖至少 2 次完整下潜-上浮
#   采样      : IMU 50Hz, mag 50Hz, pressure 10Hz, ground_truth 50Hz
#
# 录制完后运行 md5sum 写入 baseline_spec.md。

set -euo pipefail

DURATION="${1:-600}"
NS="${NS:-ug_glider}"
WS_ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
DATA_DIR="${WS_ROOT}/data"
mkdir -p "${DATA_DIR}"

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${DATA_DIR}/baseline_zigzag_${STAMP}.bag"

TOPICS=(
  "/${NS}/imu"
  "/${NS}/mag"
  "/${NS}/pressure"
  "/${NS}/ground_truth/pose"
  "/${NS}/actuator_state"
  "/${NS}/actuator_cmd"
  "/${NS}/glider_state"
  "/${NS}/cmd/depth"
  "/${NS}/cmd/heading"
  "/clock"
)

echo "[record_baseline] 录制 ${DURATION}s 到 ${BAG}"
echo "[record_baseline] topics:"
printf '  %s\n' "${TOPICS[@]}"

timeout "${DURATION}" rosbag record -O "${BAG}" "${TOPICS[@]}" || true

if [[ -f "${BAG}" ]]; then
  MD5="$(md5sum "${BAG}" | awk '{print $1}')"
  SIZE="$(du -h "${BAG}" | awk '{print $1}')"
  echo "[record_baseline] 完成: ${BAG} (${SIZE}, md5=${MD5})"
  echo "[record_baseline] 把这一行追加到 baseline_spec.md:"
  echo "  - $(basename "${BAG}") | duration=${DURATION}s | size=${SIZE} | md5=${MD5}"
else
  echo "[record_baseline] 失败：bag 文件未生成"
  exit 1
fi
