# 任务状态消息，mission_node发布，viz节点订阅
Header header

uint8 IDLE     = 0
uint8 DIVING   = 1
uint8 CLIMBING = 2
uint8 ARRIVED  = 3

uint8   state
float64 target_north        # 目标位置 NED [m]
float64 target_east
float64 distance            # 到目标水平距离 [m]
float64 dive_depth          # 当前zigzag参数
float64 climb_depth
float64 acceptance_radius
