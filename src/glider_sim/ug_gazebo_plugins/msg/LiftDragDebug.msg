# LiftDragDebug.msg
# 升阻力插件调试消息，每个字段名直接对应物理含义

std_msgs/Header header

# 力 [N]
float64 lift                # 升力大小
float64 drag                # 阻力大小

# 力向量，世界坐标系 [N]
geometry_msgs/Vector3 force_world

# 气动参数
float64 alpha               # 攻角 [rad]
float64 cl                  # 升力系数 
float64 cd                  # 阻力系数 

# 流场
float64 relative_speed      # 相对流速 [m/s]
geometry_msgs/Vector3 current_velocity   # 当前洋流速度 ENU [m/s]
