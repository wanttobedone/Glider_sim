# 水声通信协议消息
# 所有 Glider 共享 /swarm/acoustic_channel 话题
Header header

uint8 sender_id         # 发送者 ID (0=G0, 1=G1, 等等)
uint8 receiver_id       # 接收者 ID (255=广播频段)
uint8 opcode            # 操作码，指令，每个操作码代表对目标Glider的指令

float64 x               # 情报坐标，正常通信时是本机坐标，发现target是target坐标 north (NED) [m]
float64 y               # 情报坐标 east  (NED) [m]
float64 z               # 情报坐标 depth (NED) [m]

uint8 HEARTBEAT=0       # 心跳包，通信维持，常规握手，交换航位
uint8 TARGET_FOUND=1    # 发现目标
uint8 REQ_SURFACE=3     # 请求上浮中继
uint8 ACK=4             # 确认收到
uint8 MISSION_COMPLETE=5 # 任务完成
