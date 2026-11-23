#ifndef POSITION_PID_CONTROLLER__CONTROL_MATH_HPP_
#define POSITION_PID_CONTROLLER__CONTROL_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace position_pid_controller
{

constexpr double kSmallNumber = 1.0e-9;

/// 将任意角度归一化到 [-pi, pi]，用于计算 yaw 的最短转向误差。
inline double wrap_angle(double angle)
{
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("待归一化的角度必须是有限数值");
  }
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct PlanarCommand
{
  double x{0.0};
  double y{0.0};
};

/**
 * @brief 将机体系平面速度旋转到 map 坐标系。
 *
 * 机体系采用 ROS 约定：x 向前、y 向左。PID 仍以 map frame 发布，后续
 * flight_controller_bridge 会做逆旋转，因此飞控最终收到的仍是原始机体系速度。
 */
inline PlanarCommand body_command_to_map(
  double forward, double left, double current_yaw)
{
  if (!std::isfinite(forward) || !std::isfinite(left) || !std::isfinite(current_yaw)) {
    throw std::invalid_argument("机体系速度和当前 yaw 必须是有限数值");
  }

  const double cosine = std::cos(current_yaw);
  const double sine = std::sin(current_yaw);
  return {
    cosine * forward - sine * left,
    sine * forward + cosine * left};
}

inline void validate_speed_limits(double minimum, double maximum)
{
  if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
    minimum < 0.0 || maximum <= 0.0 || minimum > maximum)
  {
    throw std::invalid_argument("速度下限和上限必须满足 0 <= min <= max，且 max > 0");
  }
}

/**
 * @brief 对 x/y PID 输出实施平面最小速度与最大模长限制。
 *
 * 1. PID 矢量非零但太小时，保持 PID 给出的方向并提升到最小速度；
 * 2. PID 矢量近似为零但位置仍有误差时，改用目标方向输出最小速度；
 * 3. 最后按矢量模长限制最大速度，斜向运动也不会超过 max_xy_speed。
 */
inline PlanarCommand apply_planar_speed_limits(
  double raw_x, double raw_y,
  double error_x, double error_y,
  double min_xy_speed, double max_xy_speed)
{
  validate_speed_limits(min_xy_speed, max_xy_speed);
  if (!std::isfinite(raw_x) || !std::isfinite(raw_y) ||
    !std::isfinite(error_x) || !std::isfinite(error_y))
  {
    throw std::invalid_argument("平面速度和位置误差必须是有限数值");
  }

  double command_x = raw_x;
  double command_y = raw_y;
  double command_norm = std::hypot(command_x, command_y);

  if (command_norm < min_xy_speed) {
    if (command_norm > kSmallNumber) {
      // 原始 PID 已给出明确方向，只放大模长，不改变方向。
      const double scale = min_xy_speed / command_norm;
      command_x *= scale;
      command_y *= scale;
    } else {
      // PID 因参数或抵消关系近似输出零，使用当前位置指向目标的位置误差方向。
      const double error_norm = std::hypot(error_x, error_y);
      if (error_norm > kSmallNumber) {
        const double scale = min_xy_speed / error_norm;
        command_x = error_x * scale;
        command_y = error_y * scale;
      } else {
        command_x = 0.0;
        command_y = 0.0;
      }
    }
    command_norm = std::hypot(command_x, command_y);
  }

  if (command_norm > max_xy_speed) {
    const double scale = max_xy_speed / command_norm;
    command_x *= scale;
    command_y *= scale;
  }

  return {command_x, command_y};
}

/**
 * @brief 对 yaw PID 输出实施最小角速度和最大角速度限制。
 */
inline double apply_yaw_rate_limits(
  double raw_yaw_rate, double yaw_error,
  double min_yaw_rate, double max_yaw_rate)
{
  validate_speed_limits(min_yaw_rate, max_yaw_rate);
  if (!std::isfinite(raw_yaw_rate) || !std::isfinite(yaw_error)) {
    throw std::invalid_argument("yaw 角速度和误差必须是有限数值");
  }

  double command = raw_yaw_rate;
  if (std::abs(command) < min_yaw_rate) {
    // 小输出统一采用最短 yaw 误差方向，避免微分噪声把最小角速度推向反方向。
    if (std::abs(yaw_error) > kSmallNumber) {
      command = std::copysign(min_yaw_rate, yaw_error);
    } else {
      command = 0.0;
    }
  }

  return std::clamp(command, -max_yaw_rate, max_yaw_rate);
}

/**
 * @brief 带进入/退出双阈值的保持状态，防止最小速度在目标附近反复启停。
 */
class HysteresisHold
{
public:
  HysteresisHold(double enter_tolerance, double exit_tolerance)
  : enter_tolerance_(enter_tolerance), exit_tolerance_(exit_tolerance)
  {
    if (!std::isfinite(enter_tolerance_) || !std::isfinite(exit_tolerance_) ||
      enter_tolerance_ < 0.0 || exit_tolerance_ < enter_tolerance_)
    {
      throw std::invalid_argument("迟滞阈值必须满足 0 <= 进入阈值 <= 退出阈值");
    }
  }

  bool update(double absolute_error)
  {
    if (!std::isfinite(absolute_error) || absolute_error < 0.0) {
      throw std::invalid_argument("迟滞判断所用误差必须是非负有限数值");
    }

    if (holding_) {
      // 等于退出阈值时仍保持，只有明确漂移超过阈值才恢复运动。
      if (absolute_error > exit_tolerance_) {
        holding_ = false;
      }
    } else if (absolute_error <= enter_tolerance_) {
      holding_ = true;
    }
    return holding_;
  }

  void reset() {holding_ = false;}
  bool holding() const {return holding_;}

private:
  double enter_tolerance_;
  double exit_tolerance_;
  bool holding_{false};
};

}  // namespace position_pid_controller

#endif  // POSITION_PID_CONTROLLER__CONTROL_MATH_HPP_
