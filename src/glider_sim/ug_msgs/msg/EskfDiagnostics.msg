# 自研 ESKF 诊断信息 (Phase 1)
# 由 glider_ekf_node 发布，便于离线分析与在线监控滤波器健康度。

Header header

# 时间与步长
float64 t_last          # 最近一次 IMU predict 的传感器时间戳 (s)
float64 dt_last         # 最近一次 predict 的步长 (s)

# 深度更新统计
float64 nis_depth       # 最近一次深度更新的 NIS
uint32 accept_depth     # 累计接受次数
uint32 reject_depth     # 累计 NIS 拒绝次数

# 跳帧/复位计数 (dt 超限触发)
uint32 reset_count

# 协方差迹 (按状态块)
float64 P_trace_pos     # 位置 3 维迹 (m^2)
float64 P_trace_vel     # 速度 3 维迹 ((m/s)^2)
float64 P_trace_att     # 姿态 3 维迹 (rad^2)

# 当前 bias 估计 (FRD)
float64[3] b_g          # gyro bias (rad/s)
float64[3] b_a          # accel bias (m/s^2)
