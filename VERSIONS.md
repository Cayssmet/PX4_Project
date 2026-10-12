# 无人机导航工作区版本清单

当前工作区发布版本：**v1.1**

## 自研功能包

| 功能包 | 版本 | 说明 |
| --- | --- | --- |
| `uav_interfaces` | v1.0 | ROS 2 消息与接口定义 |
| `flight_controller_bridge` | v1.0 | 飞控串口协议、状态反馈与控制桥接 |
| `cartographer_config` | v1.0 | N10 雷达 Cartographer 配置 |
| `map_coordinate_provider` | v1.0 | 地图坐标与姿态输出 |
| `waypoint_manager` | v1.1 | D 题任务流程、横移、前追和视觉辅助阶段 |
| `position_pid_controller` | v1.1 | 固定目标及视觉相对位移控制、偏航参数校准 |
| `uav_bringup` | v1.1 | 全系统 launch 与开机自启入口 |

## 第三方功能包

第三方包保留上游 `package.xml` 版本，避免伪造或覆盖依赖版本；下表的“集成版本”表示它们已纳入当前工作区发布版本。

| 功能包 | 上游版本 | 集成版本 |
| --- | --- | --- |
| `lslidar_driver` | 1.2.0 | v1.1 |
| `lslidar_msgs` | 1.2.0 | v1.1 |
| `csm` | 0.0.0（上游清单） | v1.1 |
| `ros2_laser_scan_matcher` | 0.0.0（上游清单） | v1.1 |

## 版本规则

- `v1.0`：基础导航、建图、PID 和串口控制链路可运行版本。
- `v1.1`：针对 D 题流程及当前 PID 校准结果的版本。
- ROS 2 `package.xml` 和 CMake 使用三段式版本，例如 `1.1.0`；文档中简写为 `v1.1`。
- 后续功能修改先更新对应包版本，再同步本文件和根目录 `VERSION`。
