# 硬件抽象层 -> 控制层 的执行机构实际状态
# 反映经过动力学滤波后的真实位置

Header header

float64 battery_position     # 电池实际位置 [m]
float64 ballast_volume       # 油囊实际体积 [m^3]
float64 rudder_angle         # 尾舵实际角度 [rad]
