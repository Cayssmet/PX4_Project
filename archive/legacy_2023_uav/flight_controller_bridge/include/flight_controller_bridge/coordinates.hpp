#ifndef FLIGHT_CONTROLLER_BRIDGE__COORDINATES_HPP_
#define FLIGHT_CONTROLLER_BRIDGE__COORDINATES_HPP_

#include <cmath>

namespace flight_controller_bridge
{

/**
 * @brief 飞控机体系下的水平速度。
 *
 * 凌霄飞控的正方向已经按实机确认：
 * - x：机头前方为正；
 * - y：机体左侧为正。
 */
struct BodyVelocity
{
  double x{0.0};
  double y{0.0};
};

/**
 * @brief 将 map 坐标系速度旋转到飞控机体系。
 *
 * position_pid_controller 发布的 vx、vy 是地图坐标系速度，不能原样交给飞控。
 * 当前机头相对 map 的航向为 yaw 时，map -> body 使用逆旋转 R(-yaw)：
 *
 *   vx_body =  cos(yaw) * vx_map + sin(yaw) * vy_map
 *   vy_body = -sin(yaw) * vx_map + cos(yaw) * vy_map
 *
 * yaw=0 时两套坐标轴重合；yaw=+90° 时，地图 +x 位于机体右侧，所以
 * 对应 vy_body 为负，这也是本函数的重要测试用例。
 */
inline BodyVelocity map_velocity_to_body(
  const double vx_map, const double vy_map, const double yaw)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);

  return BodyVelocity{
    cosine * vx_map + sine * vy_map,
    -sine * vx_map + cosine * vy_map};
}

}  // namespace flight_controller_bridge

#endif  // FLIGHT_CONTROLLER_BRIDGE__COORDINATES_HPP_
