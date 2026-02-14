# 控制层 -> 硬件抽象层 的执行机构指令
# 所有字段为目标值，硬件层负责动力学过渡

Header header

float64 battery_position     # 电池质心目标位置 [m]，前正后负，范围 [-0.03385, +0.03385]
float64 ballast_volume       # 油囊目标体积 [m^3]，范围 [0, 0.001]
float64 rudder_angle         # 尾舵目标角度 [rad]，右转为正，范围 [-0.52, +0.52]
