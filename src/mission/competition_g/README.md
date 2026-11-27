# competition_g —— 2023 电赛 G 题任务层

## 赛题硬指标（已从题目 PDF 核实）

- 巡防区 **48dm × 40dm**（4.8 m × 4.0 m），左下角为原点 (0, 0)，右上角 (48, 40)，单位 dm。
- 判覆盖单位是 **8dm × 8dm 粗线框**，激光笔扫到框内任意一点即算该框被覆盖 ⇒ 共 **6 × 5 = 30 格**。
- 巡逻高度 **18dm**（1.8 m）。
- 发生火情时：先接近到水平距离 **≤5dm** 确认并亮 LED 示警 → 飞到火源正上方降到 **10dm** → 悬停 **3 s** → 抛洒灭火包（落点在以火源为圆心 **半径 3dm** 圆内）。
- 通信：巡逻期间**每秒 1 次**把无人机坐标发给消防车（1 Hz）。
- 发挥部分限时 **360 s**。

## 蛇形巡逻航线（v0.1.0 的规划结论）

取 **y = 4, 12, 20, 28, 36 dm** 五条横线（每条都穿过对应那一行 8dm 格子的中线）：

```
(0,4) -> (48,4) -> (48,12) -> (0,12) -> (0,20) -> (48,20) -> (48,28) -> (0,28) -> (0,36) -> (48,36)
```

- 总航程 **5 × 48 + 4 × 8 = 272 dm = 27.2 m**
- 0.6 m/s 约 55 s，0.8 m/s 约 44 s，1.0 m/s 约 37 s
- 纵向容差 **±4dm（±40cm）**，对 SLAM 漂移很宽容

## 任务状态机（计划）

```
IDLE -> TAKEOFF(18dm) -> PATROL(蛇形) -> [发现火情] -> APPROACH(≤5dm 确认 + LED)
     -> OVERHEAD(火源正上方) -> DESCEND(10dm) -> HOVER(3s) -> DROP(投包)
     -> REPORT(把火源坐标发给小车) -> RESUME(回到航线断点) -> ... -> RTL -> LAND
```

关键点：
- **从航线脱离到火源上方这一段由 K230 视觉导航完成**（`k230_bridge` 提供偏差与状态位），
  本包只负责「什么时候进入视觉阶段、什么时候退出、退出后回到哪个航点」。
- 火源坐标必须换算成**场地坐标（dm）**再发给小车，不能直接转发 SLAM 坐标。

## 依赖的输入话题（计划）

| 话题 | 类型 | 来源 |
|---|---|---|
| `/map/current_pose` | `geometry_msgs/PoseStamped` | `map_coordinate_provider` |
| `/map/current_pose_valid` | `std_msgs/Bool` | `map_coordinate_provider` |
| `/k230/target` | 待定 | `k230_bridge` |
| `/flight/status` | `uav_interfaces/FlightStatus` | 飞控适配层 |

## 输出话题（计划）

| 话题 | 类型 | 去向 |
|---|---|---|
| `/mission/target_waypoint` | `geometry_msgs/PoseStamped` | 位置环 PID |
| `/mission/status` | `uav_interfaces/MissionStatus` | 地面站 / 记录 |
| `/mission/fire_report` | 待定 | `pos_link`（发给小车；先在 `pos_link` 帧里加 `0x02` 火源类型） |

## 状态

- v0.1.0：**只有目录骨架**，没有节点代码。
- 下一步：先把航线生成与状态机写成纯逻辑（可单元测试、不依赖 ROS），再接话题。
