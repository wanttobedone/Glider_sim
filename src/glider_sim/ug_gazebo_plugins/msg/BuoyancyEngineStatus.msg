# BuoyancyEngineStatus.msg
# 浮力引擎状态消息（纯物理层，不含泵动力学）

std_msgs/Header header

# 体积状态 [m^3]
float64 current_volume      # 当前油囊体积
float64 neutral_volume      # 中性浮力体积 (500cc)

# 百分比表示 [0-100]
float64 volume_percent      # 当前体积百分比 (0%=空, 50%=中性, 100%=满)

# 力输出 [N]
float64 buoyancy_force      # 当前产生的额外浮力 (正=向上, 负=向下)
