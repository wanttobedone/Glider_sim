# host_replay — STM32 移植的 host 验证工具

无 ROS 依赖地在 host 上回放锁定 baseline，验证将要烧进 STM32 的同一份 core+wrapper 代码。

## 文件
- `extract_sensors.py` — bag → `sensors.csv`(原始 body-FLU) + `gt.csv`(NED 真值)
- `replay_core.cpp` — 直接调 `ug_ekf::Eskf` 的回放器（测 core 层）
- `replay_glider.cpp` — 调 STM32 C wrapper `ug_ekf_glider.h` 的回放器（测**将上板的同一代码**）
- `cross_compile_check.sh` — arm-none-eabi 交叉编译 core+wrapper，报 flash/RAM

## 用法
```bash
source <ws>/devel/setup.bash
mkdir -p /tmp/hr
python3 extract_sensors.py <ws>/data/baseline_zigzag_20260608_175201.bag /tmp/hr

INC="-I ../../include -I /usr/include/eigen3"
SRC="replay_glider.cpp ../../src/embedded/ug_ekf_glider.cpp ../../src/core/eskf.cpp"
# fp64
g++ -std=c++17 -O2 -fno-exceptions -fno-rtti $INC $SRC -o /tmp/rg64
# fp32 (上板精度)
g++ -std=c++17 -O2 -fno-exceptions -fno-rtti -DUG_EKF_USE_FP32 $INC $SRC -o /tmp/rg32
/tmp/rg32 /tmp/hr/sensors.csv 8.0 > /tmp/hr/trace.csv
```

## 已验证结论 (2026-06)
- fp64 与 fp32 轨迹差：attitude max < 0.001°、depth max < 3e-5 m
- wrapper 复现 core/ROS 结果：depth 0.0218m / roll 0.184° / pitch 0.189° / yaw 0.406°
- 增量对齐后 wrapper 运行态 RAM bss ≈ 1.4KB（无对齐大缓冲）

## 交叉编译（需工具链）
```bash
sudo apt-get install -y gcc-arm-none-eabi
bash cross_compile_check.sh
```
目标：STM32H743 Cortex-M7 / fpv5-d16 / hard-float / `-DUG_EKF_USE_FP32`。
