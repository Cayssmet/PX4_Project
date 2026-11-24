# archive/legacy_v2_cpp_frame_transforms

这里放的是 **v2.0.0 阶段的 C++ 坐标换算骨架**，已被 v3.1.0 的 Python 版取代。

## 为什么归档而不是删掉

* `frame_transforms.cpp` / `.hpp` 里的数学是**对的**，且有 12 个 gtest 断言
  （含 2 万组随机位姿的自洽性检验）。保留它，是为了将来万一要回到 C++
  （比如上行频率高到 Python 跟不上）时有现成可用的参考实现。
* `qos_profiles.hpp` 是 uXRCE-DDS 时代的 QoS 助手。DDS 路线已因带宽问题放弃，
  这份代码不会再用，但它是「PX4 的 QoS 必须 best_effort」这条结论的出处。

## 为什么归档而不是继续维护

1. **传输层换了**：v3.0.0 实测 DDS 需要 35 KB/s，而这条 CH340 线在
   460800 上一条数据都收不到；改 MAVLink 后 6.5 KB/s 就够。
2. **一份实现比两份好**：如果 C++ 换算库和 Python 节点各有一份数学，
   迟早会漂移。现在只保留 `src/flight/px4_offboard/px4_offboard/frames.py`
   一份，测试也直接对着它。
3. **这台机器没有 C++ 工具链**：`g++` / `cmake` 全都没有，那 12 个 gtest
   从来没跑过。而 Python 版的两套测试（`test_frames.py` 16 项、
   `test_mavlink_codec.py` 18 项）是**实跑通过**的。

## 想拿回来怎么办

```bash
git show v3.0.1:src/flight/px4_offboard/src/frame_transforms.cpp
```

或者在 `archive/` 里直接读 —— 文件都在。
