# uav_interfaces v1.1

`uav_interfaces` 是无人机项目的公共 ROS 2 接口包，面向 Ubuntu 24.04 和
ROS 2 Jazzy。它统一规定航点、航线、飞控命令、飞控状态和任务状态的数据格式，供
后续的航点管理、地面站通信和飞控串口桥共同使用。

这个包**不运行任何节点**，也不执行控制算法。编译它的目的，是让 ROS 2 自动生成
C++ 和 Python 可以使用的消息/服务代码。

## 一、为什么需要单独的接口包

后续的数据关系是：

```text
ground_station_bridge
        ↓ WaypointList
waypoint_manager
        ├── PoseStamped → position_pid_controller
        ├── Waypoint → flight_controller_bridge
        └── MissionStatus → 地面站和调试终端

flight_controller_bridge
        ├── FlightCommand → 串口编码 → 凌霄飞控
        └── 凌霄飞控反馈 → FlightStatus → waypoint_manager
```

如果这些功能包分别定义自己的字段，很容易出现单位、坐标方向和字段含义不一致。
`uav_interfaces` 就是它们共同遵守的数据合同。

本包不会替换已经验证成功的接口：

- `map_coordinate_provider` 仍发布 `geometry_msgs/msg/PoseStamped`；
- `position_pid_controller` 仍接收 `/pid/target_pose`；
- `position_pid_controller` 仍发布 `geometry_msgs/msg/TwistStamped`。

以后由 `waypoint_manager` 把自定义 `Waypoint` 中的 x、y、yaw 转换成 PID 需要的
`PoseStamped`。

## 二、包内文件

```text
uav_interfaces/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/
│   ├── Waypoint.msg
│   ├── WaypointList.msg
│   ├── FlightCommand.msg
│   ├── FlightStatus.msg
│   └── MissionStatus.msg
├── srv/
│   └── SetWaypointList.srv
└── test/
    └── test_generated_interfaces.cpp
```

本包没有 `src/`、`launch/` 和 `config/`，这是正常的，因为它没有可执行节点。

## 三、统一坐标与单位

所有后续功能包必须遵守：

| 数据 | 坐标系/单位 | 约定 |
|---|---|---|
| 航点 x、y | `map`，m | 地图坐标系目标位置 |
| 航点 yaw | `map`，rad | 俯视逆时针为正 |
| 目标高度 | m | 正数离地高度，不是 `map` 的 z，也不是 `vz` |
| 停留时间 | s | 完整到达航点后继续保持的时间 |
| 飞控 vx | `base_link`，m/s | 机头方向为正 |
| 飞控 vy | `base_link`，m/s | 机体左侧为正 |
| 飞控 yaw_rate | rad/s | 俯视逆时针为正 |
| 飞控 z | 上方为正 | z 轴 PID 在飞控内部完成 |

ROS 2 内部统一使用米、弧度和秒。以后编码串口帧时，
`flight_controller_bridge` 才把它们转换成 `mm`、`mm/s` 和 `mrad/s` 等定点整数。

## 四、Waypoint：单个航点

```text
uint32 id
float64 x
float64 y
float64 yaw
float64 target_height
float64 hold_time
```

- `id` 是稳定航点编号，不等同于数组下标；地面站和未来附加动作可以用它关联航点。
- `x/y/yaw` 是发给三轴 PID 的目标。
- `target_height` 是发给飞控 z 轴 PID 的正数离地目标高度。
- `hold_time` 从 x、y、yaw 和高度全部到达后开始计时。

高度方案已经固定为：

```text
数值目标高度 + control_valid + land_request
```

首版没有 `height_mode` 和高度等级。`target_height=0` 不表示屏蔽或降落；是否合法由
后续航点管理器根据可配置的最小/最大高度进行判断。

## 五、WaypointList：完整航线

```text
std_msgs/Header header
uint32 route_id
Waypoint[] waypoints
```

航线是**有顺序的航点列表**：必须先完成 `waypoints[0]`，然后才执行
`waypoints[1]`。

要求：

- `header.frame_id` 必须为 `map`；
- `route_id` 标识航线版本；
- 航点数组不能为空；
- 同一航线内的航点 `id` 应唯一；
- 航点管理器必须先验证全部航点，再整体替换旧航线。

消息定义本身只保存数据，不会自动执行这些检查；检查逻辑属于下一步的
`waypoint_manager`。

## 六、SetWaypointList：整体覆盖航线

请求：

```text
WaypointList waypoint_list
```

响应：

```text
bool accepted
string message
```

使用服务而不是只依赖普通话题，可以让地面站明确知道新航线是否成功生效。验证失败
时应返回 `accepted=false`，同时保留原航线。

服务名称将在 `waypoint_manager` 中确定，建议使用：

```text
/waypoint/set_list
```

## 七、FlightCommand：即将发送给飞控的命令

```text
std_msgs/Header header
float32 velocity_x
float32 velocity_y
float32 yaw_rate
float32 target_height
bool control_valid
bool land_request
```

`header.frame_id` 必须为 `base_link`，表示 x/y 速度已经由地图坐标系转换成凌霄飞控
机体系：前正、左正。

飞控处理规则：

| control_valid | land_request | 行为 |
|---|---|---|
| true | false | 执行平面速度、yaw_rate 和数值目标高度 |
| true | true | 忽略普通目标高度，进入独立降落流程 |
| false | 任意 | 不继续执行新的树莓派运动目标，进入约定安全处理 |

这不是实际串口帧，因此不包含帧头、帧尾、sequence 或 CRC16。它们由
`flight_controller_bridge` 的串口编码模块添加。

## 八、FlightStatus：飞控任务反馈

```text
std_msgs/Header header
bool communication_ok
bool height_reached
bool land_accepted
bool land_completed
bool failsafe
```

本消息故意不包含飞控姿态。树莓派只使用任务推进和安全判断所需状态。

当前完整航点的完成条件将是：

```text
/pid/target_reached == true
        &&
FlightStatus.height_reached == true
        &&
Waypoint.hold_time 已满足
```

## 九、MissionStatus：任务进度

任务状态常量：

| 数值 | 常量 | 含义 |
|---:|---|---|
| 0 | `WAITING_FOR_ROUTE` | 尚无合法航线 |
| 1 | `WAITING_FOR_PID` | 航线有效，但 PID 还未连接 |
| 2 | `TRACKING` | 正在追踪当前航点 |
| 3 | `DWELLING` | 已到达，正在计算停留时间 |
| 4 | `COMPLETED` | 最后一个航点已完成 |
| 5 | `ERROR` | 航线或任务发生错误 |

其余字段提供航线编号、当前数组下标、当前航点 id、航点总数、航线是否有效、目标
是否已经发送和任务是否完成。


## 十、UavStatus 与 GcsStatus：地面站ROS镜像

`UavStatus.msg` 是统一设备协议 v1.7 中固定29字节 `UAV_STATUS(0x30)` 的ROS镜像，包含session、任务阶段、相对H点坐标、高度、START后秒数和全部状态标志。

`GcsStatus.msg` 是可选15字节 `GCS_STATUS(0x31)` 心跳的ROS镜像。`communication_ok` 只表示最近1秒内是否收到合法地面站心跳；它不进入任务控制，也不能触发飞控安全状态。

这两个消息供 `gcs_serial_bridge` 发布 `/gcs/uav_status`、`/gcs/status` 和 `/gcs/link_ok`。地面站没有飞行控制权限，小车状态仍由小车直接发给地面站。
## 十一、放入树莓派工作空间

本次交付压缩包解压后，第一层就是 `uav_interfaces/`。放置结果必须是：

```text
/home/ljz/ros2_ws/
└── src/
    ├── uav_interfaces/
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── msg/
    │   └── srv/
    ├── map_coordinate_provider/
    └── position_pid_controller/
```

若压缩包位于下载目录：

```bash
unzip ~/Downloads/uav_interfaces.zip -d /home/ljz/ros2_ws/src/
```

不要出现下面的重复嵌套：

```text
/home/ljz/ros2_ws/src/uav_interfaces/uav_interfaces/package.xml
```

## 十二、安装依赖与构建

```bash
cd /home/ljz/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select uav_interfaces
source /home/ljz/ros2_ws/install/setup.bash
```

成功时应看到：

```text
Summary: 1 package finished
```

本包不会出现可以 `ros2 run` 的程序，也没有 launch 文件。

## 十三、确认接口已经生成

逐个查看：

```bash
ros2 interface show uav_interfaces/msg/Waypoint
ros2 interface show uav_interfaces/msg/WaypointList
ros2 interface show uav_interfaces/msg/FlightCommand
ros2 interface show uav_interfaces/msg/FlightStatus
ros2 interface show uav_interfaces/msg/MissionStatus
ros2 interface show uav_interfaces/srv/SetWaypointList
```

列出整个包：

```bash
ros2 interface package uav_interfaces
```

如果这些命令能显示字段与中文注释，说明接口生成成功。

## 十四、运行生成接口测试

```bash
cd /home/ljz/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select uav_interfaces
colcon test-result --verbose
```

测试会实际包含生成后的 C++ 消息/服务头文件，并检查航点顺序、飞控命令字段、任务
状态常量和航线覆盖服务的请求/响应。

## 十五、在后续 C++ 包中使用

下游包的 `package.xml` 添加：

```xml
<depend>uav_interfaces</depend>
```

`CMakeLists.txt` 添加：

```cmake
find_package(uav_interfaces REQUIRED)

ament_target_dependencies(your_node
  rclcpp
  uav_interfaces)
```

C++ 源码引用：

```cpp
#include "uav_interfaces/msg/waypoint.hpp"
#include "uav_interfaces/msg/waypoint_list.hpp"

uav_interfaces::msg::Waypoint waypoint;
waypoint.id = 1;
waypoint.x = 2.0;
waypoint.y = 1.0;
waypoint.yaw = 0.0;
waypoint.target_height = 1.5;
waypoint.hold_time = 1.0;
```

## 十六、在后续 Python 包中使用

Python 功能包的 `package.xml` 同样添加：

```xml
<exec_depend>uav_interfaces</exec_depend>
```

代码中导入：

```python
from uav_interfaces.msg import Waypoint, WaypointList
from uav_interfaces.srv import SetWaypointList
```

## 十七、未来增加新功能

如果新内容属于航点的基本属性，可以新增字段或增加 `WaypointV2.msg`。如果是拍照、
灯光、投放等独立行为，更推荐新增：

```text
WaypointAction.msg
PayloadCommand.msg
WaypointConstraint.msg
```

并使用 `route_id` 和 `Waypoint.id` 与现有航点关联。不要提前加入大量含义不明确的
`reserved1/reserved2` 字段。

修改任何 `.msg` 或 `.srv` 后，都必须：

1. 把新文件加入 `rosidl_generate_interfaces(...)`；
2. 重新构建 `uav_interfaces`；
3. 重新构建所有使用该接口的功能包；
4. 重新 source `install/setup.bash`。

最稳妥的开发阶段命令是重新构建当前工作空间：

```bash
cd /home/ljz/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

这不等于重新编写整个系统，只是让 ROS 2 为新接口重新生成代码。

## 十八、常见问题

### `ros2 interface show` 提示找不到包

通常是没有 source：

```bash
source /opt/ros/jazzy/setup.bash
source /home/ljz/ros2_ws/install/setup.bash
```

### C++ 提示找不到生成的 `.hpp`

检查下游包是否同时完成三项：

- `package.xml` 声明 `<depend>uav_interfaces</depend>`；
- CMake 执行 `find_package(uav_interfaces REQUIRED)`；
- 节点目标加入 `ament_target_dependencies(... uav_interfaces)`。

### 修改消息后终端仍显示旧字段

重新构建后必须在当前终端再次执行：

```bash
source /home/ljz/ros2_ws/install/setup.bash
```

并确认没有 source 另一个旧工作空间的 `install/setup.bash`。

### 为什么不能直接运行这个包

它只生成类型，不包含节点。下一步的 `waypoint_manager` 才会真正加载航点、提供航线
覆盖服务、向 PID 发送目标并发布任务状态。
