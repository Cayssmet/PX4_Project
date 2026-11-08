# MTF-01 光流 + 测距接入实录（PX4 1.17.0 / MicoAir743v2）

> 本文记录 2026-11-08 从零把 MicoAir MTF-01 光流测距一体传感器接到 PX4 上、并让它真正进入 EKF2 融合的全过程与坑。
> 出问题时先看第 6 节的坑清单；执行器界面相关的问题看 `QGC中文使用手册.md` 第 9 节。

---

## 1. 硬件与接线

| 项 | 值 |
|---|---|
| 型号 | MicoAir MTF-01（光流 + 激光/LED 测距一体） |
| 测距范围 | 0.02–8 m @90% 反射率(600 Lux)；0.01–5 m @70 kLux |
| 光流视场角 | 42°；测距视场角 6° |
| 光流最大速度 | 7 m/s（1 m 高度时） |
| 光流最小工作距离 | **8 cm** |
| 环境光需求 | **> 60 Lux**（太暗不工作，台灯下往往不够） |
| 接口 | LVTTL 3.3 V 串口，**115200**，8N1；数据频率 100 Hz |
| 供电 | **5 V**，平均 100 mA |

四根线：

```
MTF-01 GND  ────  飞控 GND
MTF-01 5V   ────  飞控 5V
MTF-01 Rx   ────  飞控 Tx      ★ Rx/Tx 必须交叉
MTF-01 Tx   ────  飞控 Rx
```

**本项目接的是飞控 TELEM2（= UART4 = `/dev/ttyS3`）。**

> 板级映射（MicoAir743v2 官方手册 + `boards/micoair/h743-v2/default.px4board`）：
> TELEM1=`ttyS0` · **TELEM2=`ttyS3`** · TELEM3=`ttyS4` · TELEM4=`ttyS7` · RC=`ttyS5` · GPS1=`ttyS2` · GPS2=`ttyS1` · URT6=`ttyS6`
> ⚠ UART8(TELEM4) 内部接板载蓝牙数传，默认 115200 —— 别把它当成空闲口。

---

## 2. 传感器侧设置（微空助手软件）

用「微空助手」连上 MTF-01（Type-C 或串口），在**传感器参数**页设置：

| 参数 | 值 | 说明 |
|---|---|---|
| 通信协议 | **MAVLink (PX4)** | 不要留在 `AUTO`，见坑 1 |
| MAV_SYS_ID | 1 | 默认即可，PX4 不按 sysid 过滤光流消息 |
| MAV_COMP_ID | 88 | 默认即可 |
| 光流方向 | NONE | 模块 X 轴（排线/箭头方向）朝机头时选 NONE |
| 光流缩放 | 100（×0.01 = 1.00） | 先别动，实飞标定后再调 |

**★ 改完必须点「保存到设备」，然后给 MTF-01 断电重上电。** 只在界面里选中是没用的（见坑 1）。

---

## 3. PX4 侧参数

### 3.1 串口 / MAVLink 实例（TELEM2 = MAV_1）

| 参数 | 值 | 备注 |
|---|---|---|
| `MAV_1_CONFIG` | **102**（TELEM2） | ★ 改了要重启飞控 |
| `MAV_1_MODE` | 0（Normal） | 子参数，`MAV_1_CONFIG` 打开后才会出现 |
| `SER_TEL2_BAUD` | **115200** | ★ MTF-01 固定 115200，出厂不会自动匹配 |
| `MAV_PROTO_VER` | **2** | ★ **不要为了光流改成 1**，见坑 2 |

### 3.2 EKF2 融合

| 参数 | 值 | 备注 |
|---|---|---|
| `EKF2_OF_CTRL` | 1（Enabled） | 光流融合开关，默认就是 1 |
| `EKF2_RNG_CTRL` | 1（conditional）或 2（Enabled） | 测距融合。`1` 是"仅在 <1 m/s 且 <5 m 时融合"，室内悬停够用 |
| `EKF2_HGT_REF` | **2**（Range sensor） | 高度基准。默认 1=GPS，本项目无 GPS 必须改 |

> `EKF2_HGT_REF` 不在任何 `params_*.yaml` 里，定义在 `src/modules/ekf2/module.yaml:87-104`：
> `type: enum`，`0 Barometric pressure / 1 GPS / 2 Range sensor / 3 Vision`，`default: 1`，`reboot_required: true`。

### 3.3 不需要改的

- `SENS_FLOW_ROT` / `SENS_FLOW_SCALE` —— **对 MAVLink 来源的光流完全不起作用**（见坑 3）
- `EKF2_OF_DELAY` / `EKF2_RNG_DELAY` —— 出厂 20 ms / 5 ms 够用，等实飞再调
- `MAV_SYS_ID` / `MAV_COMP_ID`

---

## 4. 验证：怎么确认"真的接上了"

按从粗到细三步走。

### 4.1 最粗：uORB 话题出现了没有

QGC → `Analyze → MAVLink Console`：

```
uorb top
```

正常应看到（本项目实测）：

| 话题 | 频率 |
|---|---|
| `distance_sensor` | 100 Hz |
| `sensor_optical_flow` | 100 Hz |
| `vehicle_optical_flow` | 51 Hz |
| `estimator_aid_src_optical_flow` | 52 Hz |
| `estimator_aid_src_rng_hgt` | 52 Hz |

> `uorb top` 是**常驻命令**，会一直刷屏，退出按回车/任意键，别等它自己结束。

### 4.2 中等：EKF2 到底融合了没有

```
listener estimator_status_flags -n 1
```

必须是 `True` 的几项：`cs_opt_flow`、`cs_rng_hgt`、`cs_tilt_align`、`cs_yaw_align`、`cs_rng_kin_consistent`。
必须是 `False` 的：全部 `reject_*`（尤其 `reject_optflow_x` / `reject_optflow_y` / `reject_hagl`）。

再看新息比值：

```
listener estimator_status -n 1
```

`vel_test_ratio`、`hgt_test_ratio` 都应远小于 1.0（本项目实测 0.030 / 0.0047）。大于 1 说明新息在超限、数据对不上。

### 4.3 最细：高度与方向对不对

```
listener vehicle_local_position -n 1
```

- `z` 在 NED 里**向下为正**，把飞机抬高 `z` 应该**变小**
- `dist_bottom` 应是 0.2 m 左右（台面高度）
- `eph` / `epv` 越小越好（本项目 epv ≈ 0.027 m）

方向判断（本项目实测）：`heading = -2.89491 rad = -165.86°`，机头前方向量在 NED 里是 `(cos h, sin h) = (-0.9704, -0.2432)`，所以**沿机头方向前移时 `x` 会减小**。实测观察一致 ⇒ 光流方向正确，不需要改 180°。

### 4.4 交叉验证：看每秒收到多少字节

```
mavlink status
```

看 instance #1 的 `rx`。≈ `32 字节/帧 × 100 Hz ≈ 3200 B/s` 是**正常**的；`tx` 1000+ B/s 也正常。若 `rx` 为 0，说明传感器根本没往飞控发东西。

---

## 5. 数据是怎么进到 PX4 的（源码依据）

`OPTICAL_FLOW_RAD` **一条消息同时带光流和测距**，`DISTANCE_SENSOR` 是另一条。PX4 两条都解析：

- `src/modules/mavlink/mavlink_receiver.cpp:747-790` `handle_message_optical_flow_rad()`
  - `:763-764` `sensor_optical_flow.pixel_flow[0] = flow.integrated_x;` / `pixel_flow[1] = flow.integrated_y;` —— **原样透传，没有旋转补偿、没有缩放**
  - `:781-785` `if (PX4_ISFINITE(flow.distance) && flow.distance >= 0.f) { sensor_optical_flow.distance_m = flow.distance; ... }` —— **测距数据也来自这条消息**
- `src/modules/mavlink/mavlink_receiver.cpp:868-902` `handle_message_distance_sensor()`
  - `ds.current_distance = dist_sensor.current_distance * 1e-2f`（cm → m）
  - `ds.orientation = dist_sensor.orientation`

⇒ 所以「协议一改对，光流和测距会同时出现」，不需要分别配置。

---

## 6. ★ 三个坑（按踩到的顺序）

### 坑 1：在微空助手里选了，但没「保存到设备」

**症状**：界面里明明显示 `MAVLink (PX4)`，飞控却一条数据都收不到；`mavlink status` 里 `rx ≈ 3213 B/s` 在动，但 `uorb top` 里没有 `distance_sensor`。

**怎么识破**：`rx 3213 B/s ÷ 100 Hz ≈ 32 字节/帧`。而 MAVLink 的 `DISTANCE_SENSOR` 一条就 40+ 字节、`OPTICAL_FLOW_RAD` 至少 52 字节 —— **32 字节对不上任何 MAVLink 帧**，说明它在发原生的 `Micolink` 协议，被 PX4 当垃圾丢掉。

**修**：在微空助手里点 **「保存到设备」**，然后给 MTF-01 **断电重上电**。`distance_sensor` 和 `sensor_optical_flow` 会同时出现。

### 坑 2：为了光流把 `MAV_PROTO_VER` 改成 1，结果执行器界面废了

**症状**：Actuators 页只剩"电机 config"，没有滑块，点电机测试报 `MAV_CMD_DO_MOTOR_TEST` 命令不支持。

**根因**：`MAV_PROTO_VER=1` 时 PX4 不再发 `COMPONENT_INFORMATION`，QGC 建不出新界面、退回旧通道，而旧的 209 命令 PX4 1.17 已删除。

**修**：把 `MAV_PROTO_VER` 改回 **2** 并重启飞控。**光流完全不受影响** —— 实测 PROTO_VER=1 与 2 期间 `DISTANCE_SENSOR` / `OPTICAL_FLOW_RAD` 都有数据。（详细证据链见 `QGC中文使用手册.md` 第 9 节。）

### 坑 3：以为方向/缩放要在 PX4 参数里调

**实际**：`SENS_FLOW_ROT` / `SENS_FLOW_SCALE` 只作用于 PX4 内置的光流驱动（PX4FLOW / PMW3901 等）。MAVLink 来源的光流在 `mavlink_receiver.cpp:763-764` 是原样透传的，**PX4 侧没有任何参数能纠正方向或缩放**。

⇒ 方向必须在 MTF-01 一侧配（微空助手的"安装方向"）；缩放要先保持 1.00，实飞标定后再调。

---

## 7. 已知限制（不要误判成故障）

| 字段 / 现象 | 含义 |
|---|---|
| `estimator_status.pos_test_ratio = nan` | **水平方向没有任何位置观测源**（无 GPS、无外部视觉）。光流只提供**速度**，位置是速度积分出来的 ⇒ **必然漂移**，室内不能长时间定点，也不能发位置设定点。**本项目走速度控制（`OffboardControlMode.velocity`）是对的。** |
| `vehicle_local_position.dist_bottom_valid = False` | `dist_bottom` 来自 PX4 的**地形估计器**（`EKF2.cpp:1470-1472`），不是测距直读。地形估计有效只影响地形跟随/精确降落，**定高悬停用不到** |
| `heading_good_for_control = False` | `EKF2.cpp:1467` = `isYawFinalAlignComplete()`，需要飞行中磁罗盘对齐才会变 True。静止在桌上很常见 |
| `innovation_fault_status_changes` 一直在涨 | 搬动/晃动飞机时新息瞬间超限、接受/拒绝来回跳产生的计数。**只要 `estimate_status` 里各 `*_test_ratio` 都远小于 1，就不是持续拒收** |
| 静止时 `x/y` 显示几十米 | EKF 实例运行很久 + 反复搬动累积出来的状态污染。**断电重启 + 静置 5 分钟**再看：漂动 < 5 cm 且 `xy_reset_counter` 不涨才是健康的 |

---

## 8. 端口占用冲突（重要，会影响后续计划）

**TELEM2 已经给 MTF-01 了**，而原计划是把它腾给树莓派的 uXRCE-DDS。

⇒ 树莓派侧要改用 **TELEM3（UART5 = `/dev/ttyS4`）**，对应参数：

```
UXRCE_DDS_CFG = TELEM3
SER_TEL3_BAUD = 921600
```

⚠ 本项目飞控当前的参数表里**只有 `SER_TEL1_BAUD` / `SER_TEL2_BAUD` / `SER_TEL4_BAUD` 三个**，没有 `SER_TEL3_BAUD`。启用 `UXRCE_DDS_CFG=TELEM3` 之后应会自动出现，若没有则说明该口在此固件上未暴露，需要换口或升级固件。

`MAV_1_CONFIG` 之后仍留给 MTF-01，不要动。
