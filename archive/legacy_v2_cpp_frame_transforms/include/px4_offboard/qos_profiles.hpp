#ifndef PX4_OFFBOARD__QOS_PROFILES_HPP_
#define PX4_OFFBOARD__QOS_PROFILES_HPP_

#include <cstddef>

#include <rclcpp/qos.hpp>

/**
 * @file qos_profiles.hpp
 * @brief PX4 uXRCE-DDS 话题专用的 QoS 配置。
 *
 * ## 为什么必须要这个文件
 *
 * PX4 侧的 QoS **不是 ROS 2 默认值**。ROS 2 默认是
 * `KEEP_LAST(10) + RELIABLE + VOLATILE`，而 PX4 用的是：
 *
 * | 方向 | 话题 | PX4 侧角色 | durability | reliability |
 * | --- | --- | --- | --- | --- |
 * | 飞控 -> 派 | `/fmu/out/*` | **发布者** | `TRANSIENT_LOCAL` | `BEST_EFFORT` |
 * | 派 -> 飞控 | `/fmu/in/*` | **订阅者** | `VOLATILE` | `BEST_EFFORT` |
 *
 * 两边不匹配时**不会报错**，而是变成"连上了但一个 topic 也收不到/发不进"——
 * 这是本项目 R7 号风险，实测中最容易被误判成"Agent 没起来"或"串口没通"。
 *
 * 所以：**所有跟 PX4 有关的话题，必须走这两个函数，不要手写 `10` 这种裸深度。**
 *
 * ## 深度怎么选
 *
 * - 订阅侧默认深度 1：PX4 侧发布频率有限（多数话题被 `rate_limit` 限到 50 Hz 或更低），
 *   而我们只需要最新值。深度大反而增加延迟和内存。
 * - 发布侧固定深度 1：`TrajectorySetpoint` / `OffboardControlMode` 只需要最新一帧，
 *   积压旧帧会让飞控执行过期指令。
 */

namespace px4_offboard
{

/// 订阅 `/fmu/out/*`（PX4 是发布者）用的 QoS：BEST_EFFORT + TRANSIENT_LOCAL。
inline rclcpp::QoS px4_subscription_qos(std::size_t depth = 1)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth))
         .best_effort()
         .transient_local();
}

/// 发布 `/fmu/in/*`（PX4 是订阅者）用的 QoS：BEST_EFFORT + VOLATILE + KEEP_LAST(1)。
inline rclcpp::QoS px4_publication_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1))
         .best_effort()
         .durability_volatile();
}

}  // namespace px4_offboard

#endif  // PX4_OFFBOARD__QOS_PROFILES_HPP_
