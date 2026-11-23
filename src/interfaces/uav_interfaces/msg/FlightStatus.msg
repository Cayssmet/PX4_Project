# 飞控返回给树莓派的最小任务状态集合。
# 本消息不包含飞控姿态；树莓派只读取完成航点和安全判断真正需要的状态。

# 线缆协议和V2枚举常量。
uint8 PROTOCOL_V1=1
uint8 PROTOCOL_V2=2

uint8 STANDBY=0
uint8 TAKEOFF=1
uint8 HEIGHT_HOLD=2
uint8 PRECISION_DESCENT=3
uint8 FINAL_LAND=4
uint8 RESET_MISSION=5

uint8 ACTION_NONE=0
uint8 ACTION_PENDING=1
uint8 ACTION_SUCCEEDED=2
uint8 ACTION_REJECTED=3
uint8 ACTION_FAILED=4

# stamp 表示飞控桥生成该状态的时间；frame_id 可以留空。
std_msgs/Header header
uint8 protocol_version

# 飞控桥是否正在持续收到格式、CRC 和时效均合法的飞控反馈帧。
bool communication_ok

# 兼容/诊断字段：飞控内部是否进入目标高度容差。
# 树莓派任务状态机不得使用本字段推进任务；正式动作以action_result为准。
bool height_reached

# 飞控是否已经接受树莓派的降落请求。
bool land_accepted

# 飞控是否已经完成着陆流程。
bool land_completed

# 飞控是否报告故障保护状态。为 true 时任务管理器不得继续切换普通航点。
bool failsafe

# 以下字段由V2状态帧提供；V1时保持默认值0/false。
uint8 current_mode
uint8 error_code
uint32 active_session_id
uint32 active_action_id

# SI单位镜像：高度m、垂直速度m/s；必须先检查height_valid。
float32 current_height
float32 vertical_velocity
uint16 height_age_ms
uint16 command_age_ms

uint8 action_result
bool height_valid
bool armed
bool airborne
bool landed
bool action_accepted
bool action_completed
