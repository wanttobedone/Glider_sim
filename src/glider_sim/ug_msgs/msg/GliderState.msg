# 状态转换节点输出：滑翔机当前状态 (NED 坐标系)
# 由 ground truth 或传感器融合得出

Header header

# 位置 (NED 坐标系) [m]
float64 north
float64 east
float64 depth                # 正值 = 水下深度

# 姿态 (欧拉角) [rad]
float64 roll                 # 横滚
float64 pitch                # 纵摇，低头为负
float64 yaw                  # 航向，北为0，顺时针为正

# 速度 (机体坐标系) [m/s]
float64 surge                # 前进速度
float64 sway                 # 横移速度
float64 heave                # 垂向速度

# 角速度 (机体坐标系) [rad/s]
float64 roll_rate
float64 pitch_rate
float64 yaw_rate
