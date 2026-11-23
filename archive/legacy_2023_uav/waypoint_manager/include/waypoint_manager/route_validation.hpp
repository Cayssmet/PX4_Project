#ifndef WAYPOINT_MANAGER__ROUTE_VALIDATION_HPP_
#define WAYPOINT_MANAGER__ROUTE_VALIDATION_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <unordered_set>

#include "uav_interfaces/msg/waypoint_list.hpp"

namespace waypoint_manager
{

/** @brief 航线检查所使用的范围限制，全部都可由节点 YAML 参数设置。 */
struct RouteLimits
{
  std::string map_frame{"map"};
  std::size_t max_waypoints{100U};
  double max_abs_x{1000.0};
  double max_abs_y{1000.0};
  double min_target_height{0.0};
  double max_target_height{20.0};
  double max_hold_time{120.0};
};

struct RouteValidationResult
{
  bool valid{false};
  std::string message;
  uav_interfaces::msg::WaypointList normalized_route;
};

/** @brief 将任意有限角度归一化到 [-pi, pi]。 */
inline double normalize_yaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

/**
 * @brief 原子地检查整条航线，并返回一份 yaw 已归一化的副本。
 *
 * 调用者只有在 valid=true 时才能替换当前航线。这样即使新航线中的最后一个
 * 航点非法，旧航线也不会被改写一半。
 */
inline RouteValidationResult validate_and_normalize_route(
  const uav_interfaces::msg::WaypointList & input,
  const RouteLimits & limits)
{
  RouteValidationResult result;
  result.normalized_route = input;

  if (limits.map_frame.empty()) {
    result.message = "内部参数 map_frame 为空";
    return result;
  }
  if (input.header.frame_id != limits.map_frame) {
    result.message = "航线 frame_id 必须为 '" + limits.map_frame + "'";
    return result;
  }
  if (input.waypoints.empty()) {
    result.message = "航线至少需要包含 1 个航点";
    return result;
  }
  if (input.waypoints.size() > limits.max_waypoints) {
    std::ostringstream stream;
    stream << "航点数量 " << input.waypoints.size() << " 超过上限 " <<
      limits.max_waypoints;
    result.message = stream.str();
    return result;
  }

  std::unordered_set<uint32_t> waypoint_ids;
  for (std::size_t index = 0; index < input.waypoints.size(); ++index) {
    const auto & waypoint = input.waypoints[index];
    const std::size_t human_index = index + 1U;

    if (!waypoint_ids.insert(waypoint.id).second) {
      std::ostringstream stream;
      stream << "第 " << human_index << " 个航点的 id=" << waypoint.id << " 与前面重复";
      result.message = stream.str();
      return result;
    }
    if (!std::isfinite(waypoint.x) || std::abs(waypoint.x) > limits.max_abs_x) {
      result.message = "第 " + std::to_string(human_index) + " 个航点的 x 非法或越界";
      return result;
    }
    if (!std::isfinite(waypoint.y) || std::abs(waypoint.y) > limits.max_abs_y) {
      result.message = "第 " + std::to_string(human_index) + " 个航点的 y 非法或越界";
      return result;
    }
    if (!std::isfinite(waypoint.yaw)) {
      result.message = "第 " + std::to_string(human_index) + " 个航点的 yaw 不是有限数";
      return result;
    }
    if (!std::isfinite(waypoint.target_height) ||
      waypoint.target_height < limits.min_target_height ||
      waypoint.target_height > limits.max_target_height)
    {
      result.message = "第 " + std::to_string(human_index) + " 个航点的目标高度越界";
      return result;
    }
    if (!std::isfinite(waypoint.hold_time) || waypoint.hold_time < 0.0 ||
      waypoint.hold_time > limits.max_hold_time)
    {
      result.message = "第 " + std::to_string(human_index) + " 个航点的停留时间非法或越界";
      return result;
    }

    result.normalized_route.waypoints[index].yaw = normalize_yaw(waypoint.yaw);
  }

  result.valid = true;
  result.message = "航线检查通过";
  return result;
}

}  // namespace waypoint_manager

#endif  // WAYPOINT_MANAGER__ROUTE_VALIDATION_HPP_
