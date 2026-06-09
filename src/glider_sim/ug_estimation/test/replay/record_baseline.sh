#!/usr/bin/env bash
# 录制 EKF 验证基线 rosbag。
#
# 关键时序（务必先录静止段再触发任务）：
#   1. 先启动 rosbag record。
#   2. 录 STATIC_SECONDS 秒静止水平段 —— 自研 ESKF 的 Phase 1 无姿态观测，
#      初始 roll/pitch/b_g 完全靠开头这段静止对齐；若一开始就在下潜，
#      初始姿态错几十度 → 重力泄漏 → 状态发散。
#   3. 再发布目标航点触发 zigzag 下潜。
#   4. 继续录到总时长 DURATION。
#
# 前置：full_sim.launch 已在另一终端启动（control:=true，滑翔机在水面 IDLE 静止）。
#
# 用法：
#   bash record_baseline.sh [duration_sec]
#   GOAL_EAST=none bash record_baseline.sh 600   # 跳过自动发点，自己用 RViz 触发
#
# 标准参数（baseline_spec.md 同步维护）：
#   namespace : ug_glider
#   任务      : zigzag, dive_depth=30, climb_depth=2
#   时长      : 默认 600 s (10 min)，覆盖至少 2 次完整下潜-上浮
#   采样      : IMU 50Hz, mag 50Hz, pressure 10Hz, ground_truth 50Hz

set -uo pipefail

DURATION="${1:-600}"
NS="${NS:-ug_glider}"
# 静止对齐窗口（秒）。需 > glider_ekf 的 static_align_seconds(默认3)，留余量。
STATIC_SECONDS="${STATIC_SECONDS:-10}"
# 任务目标点 (ENU: x=East, y=North)。设 GOAL_EAST=none 跳过自动发点。
GOAL_EAST="${GOAL_EAST:-200.0}"
GOAL_NORTH="${GOAL_NORTH:-0.0}"
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

REC_PID=""
cleanup() {
  if [[ -n "${REC_PID}" ]]; then
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[record_baseline] 录制 ${DURATION}s 到 ${BAG}"
echo "[record_baseline] topics:"
printf '  %s\n' "${TOPICS[@]}"

# 1) 先启动录制（后台），抓住起始静止段
rosbag record -O "${BAG}" "${TOPICS[@]}" __name:=baseline_recorder &
REC_PID=$!
echo "[record_baseline] rosbag record 已启动 (pid=${REC_PID})"

# 2) 录静止段
echo "[record_baseline] 捕获 ${STATIC_SECONDS}s 静止对齐段（请确认滑翔机此刻在水面静止）..."
sleep "${STATIC_SECONDS}"

# 3) 发目标点触发 zigzag
if [[ "${GOAL_EAST}" != "none" ]]; then
  echo "[record_baseline] 发布任务目标点 ENU(x=${GOAL_EAST}, y=${GOAL_NORTH}) 触发 zigzag"
  rostopic pub -1 "/${NS}/move_base_simple/goal" geometry_msgs/PoseStamped \
    "{header: {frame_id: '${NS}/odom'}, pose: {position: {x: ${GOAL_EAST}, y: ${GOAL_NORTH}, z: 0.0}, orientation: {w: 1.0}}}" \
    || echo "[record_baseline] 警告: 目标点发布失败，确认 full_sim 已带 control:=true 启动"
  timeout 15 rostopic echo -n1 "/${NS}/cmd/depth" >/dev/null 2>&1 \
    && echo "[record_baseline] 任务已启动 (检测到 /cmd/depth)" \
    || echo "[record_baseline] 警告: 15s 内未见 /cmd/depth，任务可能未启动"
else
  echo "[record_baseline] GOAL_EAST=none：请自行用 RViz 2D Nav Goal 触发任务"
fi

# 4) 录到总时长
REMAIN=$(( DURATION - STATIC_SECONDS ))
if (( REMAIN > 0 )); then
  echo "[record_baseline] 继续录制剩余 ${REMAIN}s ..."
  sleep "${REMAIN}"
fi

# 收尾
cleanup
REC_PID=""
sleep 1

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
