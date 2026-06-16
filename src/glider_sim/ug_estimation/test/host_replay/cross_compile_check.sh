#!/usr/bin/env bash
# arm-none-eabi 交叉编译 core + STM32 wrapper，报告 flash/RAM 占用。
# 目标：STM32H743 Cortex-M7 / fpv5-d16 / hard-float / fp32。
#
# 前置（若未装工具链）：
#   sudo apt-get install -y gcc-arm-none-eabi
# Eigen 头（host 已有）：/usr/include/eigen3
#
# 用法: bash cross_compile_check.sh [eigen_include_dir]

set -uo pipefail
EIGEN="${1:-/usr/include/eigen3}"
UG_INC="$(cd "$(dirname "$0")/../../include" && pwd)"
SRC_DIR="$(cd "$(dirname "$0")/../../src" && pwd)"

CXX=arm-none-eabi-g++
if ! command -v "$CXX" >/dev/null 2>&1; then
  echo "[cross] 未找到 $CXX。请先安装："
  echo "        sudo apt-get install -y gcc-arm-none-eabi"
  exit 1
fi
echo "[cross] $($CXX --version | head -1)"

# STM32H743 标志（与板载 Makefile 一致）
MCU="-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard"
CXXFLAGS="-std=c++17 -Os -ffunction-sections -fdata-sections \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics \
  -DUG_EKF_USE_FP32 -DEIGEN_NO_MALLOC -DEIGEN_DONT_VECTORIZE \
  -I${UG_INC} -I${EIGEN}"

OUT=/tmp/ug_ekf_arm
mkdir -p "$OUT"
echo "[cross] 编译 core ..."
$CXX $MCU $CXXFLAGS -c "${SRC_DIR}/core/eskf.cpp" -o "${OUT}/eskf.o" || exit 1
echo "[cross] 编译 wrapper ..."
$CXX $MCU $CXXFLAGS -c "${SRC_DIR}/embedded/ug_ekf_glider.cpp" -o "${OUT}/wrapper.o" || exit 1

echo ""
echo "[cross] === 占用 (Cortex-M7 fp32) ==="
arm-none-eabi-size "${OUT}/eskf.o" "${OUT}/wrapper.o"
echo ""
echo "  text=flash 代码, data=已初始化(flash+RAM), bss=零初始化 RAM"
echo "  注：滤波器运行态 RAM 主要是静态 Eskf(~1.1KB) + 对齐缓冲(可调 kAlignCap)"
