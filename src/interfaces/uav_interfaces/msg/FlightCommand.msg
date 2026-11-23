# 树莓派内部已经整理完成、准备交给飞控串口编码器的控制命令。
#
# 注意：这不是实际串口帧。帧头、帧尾、sequence 和 CRC16 属于
# flight_controller_bridge 内部的线缆协议，不放在 ROS 2 消息中。

# 线缆协议版本。V1镜像为1，V2镜像为2。
uint8 PROTOCOL_V1=1
uint8 PROTOCOL_V2=2

uint8 STANDBY=0
uint8 TAKEOFF=1
uint8 HEIGHT_HOLD=2
uint8 PRECISION_DESCENT=3
uint8 FINAL_LAND=4
uint8 RESET_MISSION=5

# header.frame_id 必须是 base_link；stamp 表示命令生成时间。
std_msgs/Header header
uint8 protocol_version
uint8 control_mode
uint32 session_id
uint32 action_id

# 凌霄飞控机体系速度：x 前方为正、y 左侧为正，单位 m/s。
float32 velocity_x
float32 velocity_y

# 绕 z 轴的目标角速度，单位 rad/s；从上方俯视时逆时针为正。
float32 yaw_rate
# V2精密下降使用的机体系垂直速度，向上为正，单位m/s；V1和其他模式为0。
float32 velocity_z


# 正数离地目标高度，单位 m。飞控用本地激光测量完成 z 轴 PID。
float32 target_height

# true 表示本周期速度、目标高度和上游定位均有效。
# false 时飞控不得继续执行新的树莓派运动目标，应进入约定的安全处理。
bool control_valid

# true 表示请求飞控进入独立降落流程。
# 降落期间飞控应忽略普通 target_height，而不能把 target_height=0 当作降落。
bool land_request
