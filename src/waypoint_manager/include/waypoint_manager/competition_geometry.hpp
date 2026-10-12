#ifndef WAYPOINT_MANAGER__COMPETITION_GEOMETRY_HPP_
#define WAYPOINT_MANAGER__COMPETITION_GEOMETRY_HPP_

#include <cmath>

namespace waypoint_manager
{

struct PlanarTarget
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * @brief 把以起飞时机体为基准的“前向/左向”位移换算到 map 坐标系。
 *
 * forward_m 为机头前方正方向；left_m 为机体左侧正方向。这样比赛现场只需修改
 * lateral_offset_m 的正负号，不需要改代码或重新定义地图坐标轴。
 */
inline PlanarTarget body_relative_target(
  double start_x, double start_y, double start_yaw,
  double forward_m, double left_m)
{
  PlanarTarget result;
  result.x = start_x + std::cos(start_yaw) * forward_m -
    std::sin(start_yaw) * left_m;
  result.y = start_y + std::sin(start_yaw) * forward_m +
    std::cos(start_yaw) * left_m;
  result.yaw = start_yaw;
  return result;
}

}  // namespace waypoint_manager

#endif  // WAYPOINT_MANAGER__COMPETITION_GEOMETRY_HPP_
