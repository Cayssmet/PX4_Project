# MicroXRCEAgent 开机自启服务

这个目录里只有一个文件：`micro-xrce-agent.service`。
它让树莓派把 `MicroXRCEAgent` 当成**系统服务**来跑，而不是在 SSH 窗口里手动敲命令。

---

## 1. systemd 服务到底是什么

### 一句话
> **systemd 是 Linux 的「进程管家」，服务文件是给管家看的一张工单：跑什么程序、以谁的身份、挂了怎么办、要不要开机就跑。**

### 不写服务文件会怎样（你现在就是这种状态）
```bash
cd ~/Micro-XRCE-DDS-Agent/build
LD_LIBRARY_PATH=$PWD ./MicroXRCEAgent serial --dev /dev/flight_controller -b 460800
```
这样跑出来的进程有三个毛病：

| 毛病 | 后果 |
|---|---|
| 属于你的 SSH 会话 | **SSH 一断，Agent 跟着死**（这正是 11-17 那次「树莓派掉线后一切都停了」的原因之一） |
| 没有单实例保护 | 谁都能再起一个 → **两个 Agent 抢同一个串口** → 数据全毁 |
| 飞控没插就退出 | 需要人再手动起一次，容易起重复 |

### 写成服务之后
- 树莓派**开机自动**跑
- 进程挂了（飞控拔了、USB 掉线）**自动重来**，插回来就恢复
- 只能用 `systemctl` 起停，**天然只有一个实例**
- 日志统一进 `journalctl`，SSH 断开也不丢

---

## 2. 文件里每一行在说什么

```ini
[Unit]                                    # 这一段描述"这个服务是什么"
Description=...                           # systemctl status 里显示的名字
After=network.target                      # 网络起来之后再启动（顺序，不是依赖）
StartLimitIntervalSec=0                   # ★ 永不放弃重启

[Service]                                 # 这一段描述"怎么跑"
Type=exec                                 # 主进程就是 ExecStart 那个程序本身
User=ljz                                  # 以 ljz 身份跑（不要用 root）
Group=dialout                             # 需要这个组才能读写 /dev/ttyUSB0
WorkingDirectory=.../build                # 工作目录
Environment=LD_LIBRARY_PATH=.../build     # Agent 没有 make install，库在 build 里
ExecStartPre=-/usr/bin/pkill -x MicroXRCEAgent   # ★ 启动前清掉残留，防双实例
ExecStart=.../MicroXRCEAgent serial --dev /dev/flight_controller -b 460800
Restart=always                            # 退出就重启
RestartSec=3                              # 隔 3 秒
TimeoutStopSec=3                          # 停止时最多等 3 秒
StandardOutput=journal                    # 日志进 journald

[Install]
WantedBy=multi-user.target                # 开机时在多用户模式启动
```

### 为什么 `ExecStartPre` 那行开头要有个 `-`
`-` 表示「这行命令失败不算错误」。
如果当前没有残留 Agent，`pkill` 会返回非 0，加上 `-` 才不会导致整个服务启动失败。

### 为什么 `StartLimitIntervalSec=0` 很关键
飞控不插电时，Agent 一启动就发现 `/dev/flight_controller` 不存在，立刻退出。
systemd 默认「10 秒内失败 5 次就彻底放弃」，那样你插上飞控也不会自动恢复。
设成 `0` = 永远重试。

---

## 3. 安装（在树莓派上执行）

把本目录的 `micro-xrce-agent.service` 复制到树莓派，然后：

```bash
# 1) 停掉现在手动跑的那个（如果还在）
pkill -x MicroXRCEAgent

# 2) 安装服务文件
sudo cp micro-xrce-agent.service /etc/systemd/system/

# 3) 让 systemd 读一遍新文件
sudo systemctl daemon-reload

# 4) 设为开机自启，并立刻启动
sudo systemctl enable --now micro-xrce-agent

# 5) 看状态（应该看到 active (running)）
systemctl status micro-xrce-agent --no-pager
```

### 如果是从 Windows 传文件
```powershell
scp D:\09_PX4\PX4_Project\deploy\systemd\micro-xrce-agent.service raspberrypi:/tmp/
```
然后在树莓派上 `sudo cp /tmp/micro-xrce-agent.service /etc/systemd/system/`。

---

## 4. 验证

```bash
# 只应该有一个进程
pgrep -c -x MicroXRCEAgent          # 期望输出 1
pgrep -a MicroXRCEAgent

# 日志（Ctrl+C 退出）
journalctl -u micro-xrce-agent -f
```
日志里应该看到：
```
init | running...            | fd: 3
create_client   | create         | client_key: 0x00000001, session_id: 0x81
establish_session | session established | client_key: 0x00000001, address: 1
create_participant | participant created | client_key: 0x00000001, participant_id: 0x001(1)
```

再确认 ROS 2 侧能看到飞控：
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep -c fmu        # 期望 65 左右
```

---

## 5. 常用命令

| 想做的事 | 命令 |
|---|---|
| 看状态 | `systemctl status micro-xrce-agent --no-pager` |
| 看实时日志 | `journalctl -u micro-xrce-agent -f` |
| 重启（改了波特率后） | `sudo systemctl restart micro-xrce-agent` |
| 临时停 | `sudo systemctl stop micro-xrce-agent` |
| 开机不自启 | `sudo systemctl disable micro-xrce-agent` |
| 确认是否开机自启 | `systemctl is-enabled micro-xrce-agent` |

---

## 6. 排障

| 现象 | 原因 | 处理 |
|---|---|---|
| `status` 里 `activating (auto-restart)` 一直循环 | 飞控没插 / `/dev/flight_controller` 不存在 | 插上飞控；用 `ls -l /dev/flight_controller` 确认符号链接在 |
| 启动成功但 `ros2 topic list` 没有 `fmu` | 飞控侧 `UXRCE_DDS_CFG` 没设成 TELEM3(103)，或 `SER_TEL3_BAUD` 与这里的 `-b` 不一致 | 两边波特率必须**完全相同**：现在都是 **460800** |
| 日志里刷 `deserialization error` / 大段乱码 | 线材信号质量差（460800 已接近这条线的极限） | 焊死 CH340 侧杜邦针、并一根 GND |
| `pgrep -c -x MicroXRCEAgent` 大于 1 | 有别的启动方式在同时跑（旧 SSH 窗口、计划任务） | `pkill -x MicroXRCEAgent` 后 `sudo systemctl restart micro-xrce-agent` |
| 改了波特率后不生效 | 飞控侧 `SER_TEL3_BAUD` 改了但 XRCE 模块没重启 | `uxrce_dds_client stop` 再 `uxrce_dds_client start -t serial -d /dev/ttyS4 -b 460800`（或重启飞控） |

---

## 7. 相关事实（实测记录）

- Agent 编译位置：`~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`（70,528 B，源码 tag `v2.4.3`）
- **没有 `sudo make install`**，所以必须靠 `WorkingDirectory` + `LD_LIBRARY_PATH` 指向 build 目录
- 工作模式：`serial` 传输 + 默认 `dds`（Fast DDS）中间件，`verbose_level: 4`
- 串口设备：`/dev/flight_controller` → `/dev/ttyUSB0`（udev 规则按 USB 物理路径绑定，**必须插回原来那个 USB 口**）
- 460800 实测：`vehicle_attitude` 56–58 Hz、`vehicle_local_position_v1` 4.7 Hz、`vehicle_status_v1` 0.6 Hz、话题 65 条
- 115200 实测（对照）：`vehicle_attitude` 1.2 Hz、`vehicle_local_position_v1` 0.7 Hz
