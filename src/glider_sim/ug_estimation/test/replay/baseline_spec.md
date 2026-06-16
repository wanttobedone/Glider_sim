# EKF Baseline rosbag 规范

EKF 自研估计器的所有阶段验证（Phase 1–5）都使用同一份 baseline rosbag 离线回放。
确保 "估计行为变化只来自代码改动，不来自不同仿真过程"。

bag 文件**不入 git**（见仓库根 `.gitignore`），存放在 `<workspace>/data/`。

---

## 标准录制场景

| 项 | 值 |
|---|---|
| 仿真世界 | `gazebo_dave_ocean.launch` (uuv_dave 海洋世界) |
| 滑翔机命名空间 | `ug_glider` |
| 初始位姿 | `x=0, y=0, z=-0.126`，`yaw=π/2`（朝向 NED 北） |
| 控制模式 | CASCADE |
| 任务 | zigzag |
| `dive_depth` | 30.0 m |
| `climb_depth` | 2.0 m |
| `mission_rate` | 1.0 Hz |
| 录制时长 | ≥ 600 s（覆盖至少 2 次完整 dive/climb 周期） |

## 录制话题

| 话题 | 频率 | 说明 |
|---|---|---|
| `/ug_glider/imu` | 50 Hz | sensor_msgs/Imu，body-FLU；EKF 只读 ω/a，不读 orientation |
| `/ug_glider/mag` | 50 Hz | sensor_msgs/MagneticField，body-FLU |
| `/ug_glider/pressure` | 10 Hz | sensor_msgs/FluidPressure，深度从压力推 |
| `/ug_glider/ground_truth/pose` | 50 Hz | nav_msgs/Odometry，仿真真值（验证用） |
| `/ug_glider/actuator_state` | 20 Hz | 后续模型辅助 EKF 用 |
| `/ug_glider/actuator_cmd` | 20 Hz | |
| `/ug_glider/glider_state` | 20 Hz | 当前 robot_localization 输出，对照用 |
| `/ug_glider/cmd/depth` `cmd/heading` | 1 Hz | mission 决策记录 |
| `/clock` | — | sim time 必录 |

## 录制流程

终端 1（启动仿真，**不**启动 mission，留待录制脚本）：

```
roslaunch ug_description full_sim.launch \
    estimation:=true control:=true \
    use_ground_truth:=false \
    rviz:=false dashboard:=false
```

终端 2：

```
cd <workspace>/src/glider_sim/ug_estimation/test/replay
bash record_baseline.sh 600
```

录完后把脚本输出的一行追加到本文件 §"已锁定基线"。

## 已锁定基线

**锁定基线 (Phase 1/2 回归基准)：**

| 文件 | 时长 | 大小 | md5 |
|---|---|---|---|
| `baseline_zigzag_20260608_175201.bag` | 523 s | 81 MB | `9ed2c00445f6d3a268f9c1fe01d12562` |

### 录制条件
- 仿真世界 `gazebo_dave_ocean.launch`，namespace `ug_glider`
- **真实中端 MEMS IMU 参数**（xacro：陀螺 bias_stddev 0.002、加计 0.05，dynamic_bias 已减小）
- 录制时序：先录 ~10s **静止水平段**（供 staticAlign），再发 `move_base_simple/goal` 触发 zigzag
- 任务 zigzag：dive_depth=30 m，climb_depth=2 m
- 起始 sim 时间 12.9 s，时间流单调干净（0 回退）

### Phase 1 验收结果（ug_eskf，headless 单次干净回放）
| 指标 | 实测 | 阈值 | 结论 |
|---|---|---|---|
| depth RMSE | 0.022 m | ≤ 0.05 | PASS |
| roll RMSE | 0.22° | ≤ 1° | PASS |
| pitch RMSE | 0.19° | ≤ 1° | PASS |
| NIS_depth 合规率 | 100% | ≥ 90% | PASS |

### Phase 2 验收结果（+ 磁力计 yaw，同一基线 headless 回放）
| 指标 | 实测 | 阈值 | 结论 |
|---|---|---|---|
| depth RMSE | 0.022 m | ≤ 0.05 | PASS |
| roll RMSE | 0.18° | ≤ 1° | PASS |
| pitch RMSE | 0.19° | ≤ 1° | PASS |
| yaw RMSE | 0.40° | ≤ 2° | PASS |
| yaw 漂移 | -0.036°/min | 无漂移 | PASS |
| mag 接受率 | 100% | ≥ 90% | PASS |

> mag yaw 更新只观测 Down 轴（H = e_D^T·R_NB），不破坏 accel 找平对 roll/pitch 的约束。
> 初始 yaw 由 staticAlign 用静止段 mag 恢复（替代 init_yaw_ned_rad 硬编码）。

> Phase 2 及后续任何 core 改动，必须用本基线回归，保证 depth/roll/pitch 不退化。
> 评估务必用修复后的 `run_phase1_eval.sh`（含残留进程强清理），避免时间流污染。

### 历史教训
- 录制必须**先静止后机动**：Phase 1 无姿态绝对观测，靠开头静止段对齐；从下潜中开录会导致初始姿态错几十度而发散。
- 评估链路若有残留 `rosbag play/record` 进程叠加，会污染输出 bag 时间戳（出现回退/回环），表现为"EKF 发散"假象。务必单次干净回放。

## 注意事项

- `imu0_remove_gravitational_acceleration` 在 robot_localization 里仍开启，但**自研 EKF 直接吃 raw 加速度（含重力）**。所以 bag 录的是 raw。
- 自研 EKF **不订阅** `imu.orientation` 字段（硬约束 C2）。bag 里有这字段不影响，是 robot_localization 的输入。
- `data/` 下若有多份 bag，所有阶段验证以 baseline_spec.md "已锁定基线"指向的 md5 为准；不要随机选 bag 跑。
- 替换 baseline 时同步更新 md5 行。
