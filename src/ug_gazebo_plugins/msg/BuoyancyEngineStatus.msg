# BuoyancyEngineStatus.msg
# 浮力引擎（油囊）状态消息

std_msgs/Header header

# 体积状态 [m^3]
float64 current_volume      # 当前油囊体积
float64 target_volume       # 目标油囊体积
float64 neutral_volume      # 中性浮力体积 (500cc)

# 百分比表示 [0-100]
float64 volume_percent      # 当前体积百分比 (0%=空, 50%=中性, 100%=满)

# 力输出 [N]
float64 buoyancy_force      # 当前产生的额外浮力 (正=向上, 负=向下)

# 泵状态
bool is_at_target           # 是否已到达目标体积
bool is_pumping             # 油泵是否正在工作
int8 pump_direction         # 泵方向: 1=充油(上浮), -1=排油(下沉), 0=停止
