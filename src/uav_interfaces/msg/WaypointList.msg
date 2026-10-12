# 一条完整航线，即“按照数组顺序执行的一组航点”。
# 航点数组是有顺序的：waypoints[0] 完成后才执行 waypoints[1]。

# header.frame_id 必须是 map；stamp 表示这份航线被创建或接收的时间。
std_msgs/Header header

# 航线版本编号。地面站每次下发新航线时应使用新的编号，便于识别覆盖关系。
uint32 route_id

# 有序航点列表。航点管理器必须在整条列表全部验证通过后再原子替换旧航线。
uav_interfaces/Waypoint[] waypoints
