# 任务状态机发给飞控桥的V2模式/session/action指令。
# 水平速度仍由PID/control_mux提供，本消息只决定飞控模式和垂直动作。

uint8 STANDBY=0
uint8 TAKEOFF=1
uint8 HEIGHT_HOLD=2
uint8 PRECISION_DESCENT=3
uint8 FINAL_LAND=4
uint8 RESET_MISSION=5

std_msgs/Header header

# 必须取上方六种模式之一。
uint8 control_mode

# 一次完整比赛使用同一个非零session；无会话安全状态使用0。
uint32 session_id

# 连续模式必须为0；离散动作在同一session内严格递增且必须非零。
uint32 action_id

# PRECISION_DESCENT使用，向上为正，单位m/s；其他模式必须为0。
float32 vertical_velocity

# TAKEOFF/HEIGHT_HOLD为目标高度；PRECISION_DESCENT为下降安全下限，单位m。
float32 target_height

# false时不能建立session或触发新动作，连续运动输出必须撤销。
bool control_valid
