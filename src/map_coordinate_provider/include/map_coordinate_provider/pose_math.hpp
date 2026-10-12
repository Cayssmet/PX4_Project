#ifndef MAP_COORDINATE_PROVIDER__POSE_MATH_HPP_
#define MAP_COORDINATE_PROVIDER__POSE_MATH_HPP_

#include <cmath>

namespace map_coordinate_provider
{

/**
 * @brief 归一化后的四元数以及由它计算得到的 yaw。
 *
 * PoseStamped 必须继续保存完整四元数，不能只把 yaw 填进 orientation.z。
 * yaw 只用于日志和测试；新版 PID 会从完整四元数中自行提取 yaw。
 */
struct NormalizedQuaternionYaw
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
  double yaw{0.0};
};

/// x、y、z 三个位置分量必须全部为有限数值。
inline bool position_is_finite(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/// 限制局部比赛地图中的绝对水平距离，避免稳定但完全错误的SLAM轨迹被重新接纳。
inline bool absolute_position_is_plausible(
  const double x, const double y, const double max_distance_m)
{
  if (!std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(max_distance_m) || max_distance_m <= 0.0)
  {
    return false;
  }
  return std::hypot(x, y) <= max_distance_m;
}

inline double wrapped_angle_difference(const double current, const double previous)
{
  return std::atan2(std::sin(current - previous), std::cos(current - previous));
}

inline bool pose_motion_is_plausible(
  const double previous_x, const double previous_y, const double previous_yaw,
  const double current_x, const double current_y, const double current_yaw,
  const double elapsed_seconds, const double max_step_m,
  const double max_yaw_step_rad, const double max_linear_speed_m_s,
  const double max_yaw_rate_rad_s)
{
  if (!std::isfinite(previous_x) || !std::isfinite(previous_y) ||
    !std::isfinite(previous_yaw) || !std::isfinite(current_x) ||
    !std::isfinite(current_y) || !std::isfinite(current_yaw) ||
    !std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0)
  {
    return false;
  }

  const double distance = std::hypot(current_x - previous_x, current_y - previous_y);
  const double yaw_change = std::abs(wrapped_angle_difference(current_yaw, previous_yaw));
  if (distance > max_step_m || yaw_change > max_yaw_step_rad) {
    return false;
  }
  if (elapsed_seconds <= 1.0e-9) {
    return distance <= 1.0e-9 && yaw_change <= 1.0e-9;
  }
  return distance / elapsed_seconds <= max_linear_speed_m_s &&
         yaw_change / elapsed_seconds <= max_yaw_rate_rad_s;
}

/**
 * @brief 检查、归一化四元数并计算 yaw。
 * @return 输入有效时返回 true；全零、NaN 或 Inf 四元数返回 false。
 */
inline bool normalize_quaternion_and_get_yaw(
  double x, double y, double z, double w,
  NormalizedQuaternionYaw & result)
{
  if (!std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(z) || !std::isfinite(w))
  {
    return false;
  }

  const double norm = std::hypot(std::hypot(x, y), std::hypot(z, w));
  if (!std::isfinite(norm) || norm < 1.0e-12) {
    return false;
  }

  result.x = x / norm;
  result.y = y / norm;
  result.z = z / norm;
  result.w = w / norm;

  // 标准四元数转 yaw 公式，结果自然位于 [-pi, pi]。
  const double sin_yaw = 2.0 * (result.w * result.z + result.x * result.y);
  const double cos_yaw =
    1.0 - 2.0 * (result.y * result.y + result.z * result.z);
  result.yaw = std::atan2(sin_yaw, cos_yaw);
  return std::isfinite(result.yaw);
}

}  // namespace map_coordinate_provider

#endif  // MAP_COORDINATE_PROVIDER__POSE_MATH_HPP_
