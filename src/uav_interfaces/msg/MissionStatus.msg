# 航点管理器对外发布的任务进度快照。

# 任务状态常量。使用数字常量而不是自由字符串，可避免不同节点拼写不一致。
uint8 WAITING_FOR_ROUTE=0
uint8 WAITING_FOR_PID=1
uint8 TRACKING=2
uint8 DWELLING=3
uint8 COMPLETED=4
uint8 ERROR=5

# stamp 表示状态生成时间；frame_id 通常填写 map。
std_msgs/Header header

# 当前航线编号。
uint32 route_id

# 当前航点在 WaypointList 数组中的下标，从 0 开始。
uint32 current_index

# 当前航点自身的稳定 id。没有活动航点时，应结合 state 和 route_valid 判断。
uint32 current_waypoint_id

# 当前航线包含的航点总数。
uint32 waypoint_count

# 当前状态，取值必须是本消息上方定义的状态常量之一。
uint8 state

# 整条航线是否已通过验证并可用于执行。
bool route_valid

# 当前航点是否已经成功发送给 position_pid_controller。
bool target_sent

# 最后一个航点的平面、yaw、高度和停留时间均已完成时为 true。
bool mission_completed
