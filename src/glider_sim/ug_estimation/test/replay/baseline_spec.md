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

> 注：第一次正式 Phase 1 验证开始前在此追加。

| 文件 | 时长 | 大小 | md5 |
|---|---|---|---|
| _待录制_ | | | |

## 注意事项

- `imu0_remove_gravitational_acceleration` 在 robot_localization 里仍开启，但**自研 EKF 直接吃 raw 加速度（含重力）**。所以 bag 录的是 raw。
- 自研 EKF **不订阅** `imu.orientation` 字段（硬约束 C2）。bag 里有这字段不影响，是 robot_localization 的输入。
- `data/` 下若有多份 bag，所有阶段验证以 baseline_spec.md "已锁定基线"指向的 md5 为准；不要随机选 bag 跑。
- 替换 baseline 时同步更新 md5 行。
