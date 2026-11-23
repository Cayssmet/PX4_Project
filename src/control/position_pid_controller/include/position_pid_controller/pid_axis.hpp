#ifndef POSITION_PID_CONTROLLER__PID_AXIS_HPP_
#define POSITION_PID_CONTROLLER__PID_AXIS_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace position_pid_controller
{

/**
 * @brief 单个轴的 PID 计算器。
 *
 * 该类不依赖 ROS 2，只处理数值计算，因此可以被单元测试直接验证。
 * x、y、yaw 三个方向各自拥有一个实例，彼此的积分和微分状态互不影响。
 */
class PidAxis
{
public:
  PidAxis(
    double kp, double ki, double kd,
    double integral_limit, double output_limit,
    double derivative_filter_alpha)
  : kp_(kp),
    ki_(ki),
    kd_(kd),
    integral_limit_(integral_limit),
    output_limit_(output_limit),
    derivative_filter_alpha_(derivative_filter_alpha)
  {
    if (!std::isfinite(kp_) || !std::isfinite(ki_) || !std::isfinite(kd_)) {
      throw std::invalid_argument("PID 增益必须是有限数值");
    }
    if (!std::isfinite(integral_limit_) || integral_limit_ < 0.0) {
      throw std::invalid_argument("积分限幅必须是非负有限数值");
    }
    if (!std::isfinite(output_limit_) || output_limit_ < 0.0) {
      throw std::invalid_argument("输出限幅必须是非负有限数值");
    }
    if (!std::isfinite(derivative_filter_alpha_) ||
      derivative_filter_alpha_ < 0.0 || derivative_filter_alpha_ >= 1.0)
    {
      throw std::invalid_argument("微分滤波系数必须位于 [0, 1) 区间");
    }
  }

  /**
   * @brief 使用“目标值减当前值”得到的误差计算一次 PID 输出。
   * @param error 本轴当前误差。
   * @param dt_sec 距上一次有效计算经过的秒数，必须大于 0。
   */
  double update_error(double error, double dt_sec)
  {
    if (!std::isfinite(error) || !std::isfinite(dt_sec) || dt_sec <= 0.0) {
      throw std::invalid_argument("PID 误差和时间间隔必须有效，且 dt 必须大于 0");
    }

    // 积分项先累积，再进行对称限幅，避免长时间误差造成积分无限增长。
    integral_ += error * dt_sec;
    integral_ = clamp_symmetric(integral_, integral_limit_);

    // 第一次计算没有历史误差，此时微分项按 0 处理，避免启动瞬间产生尖峰。
    const double raw_derivative =
      has_previous_error_ ? (error - previous_error_) / dt_sec : 0.0;

    // 一阶低通：alpha 是旧滤波值的权重。alpha 越大，微分越平滑但响应越慢。
    filtered_derivative_ =
      derivative_filter_alpha_ * filtered_derivative_ +
      (1.0 - derivative_filter_alpha_) * raw_derivative;

    previous_error_ = error;
    has_previous_error_ = true;

    const double raw_output =
      kp_ * error + ki_ * integral_ + kd_ * filtered_derivative_;
    return clamp_symmetric(raw_output, output_limit_);
  }

  /// 清空积分、微分滤波值和历史误差。
  void reset()
  {
    integral_ = 0.0;
    filtered_derivative_ = 0.0;
    previous_error_ = 0.0;
    has_previous_error_ = false;
  }

  double integral() const {return integral_;}
  double filtered_derivative() const {return filtered_derivative_;}
  bool has_previous_error() const {return has_previous_error_;}

private:
  static double clamp_symmetric(double value, double limit)
  {
    return std::clamp(value, -limit, limit);
  }

  double kp_;
  double ki_;
  double kd_;
  double integral_limit_;
  double output_limit_;
  double derivative_filter_alpha_;

  double integral_{0.0};
  double filtered_derivative_{0.0};
  double previous_error_{0.0};
  bool has_previous_error_{false};
};

}  // namespace position_pid_controller

#endif  // POSITION_PID_CONTROLLER__PID_AXIS_HPP_
