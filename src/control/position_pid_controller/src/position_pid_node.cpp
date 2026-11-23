#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "position_pid_controller/control_math.hpp"
#include "position_pid_controller/pid_axis.hpp"

namespace position_pid_controller
{

/**
 * @brief 无人机平面双模式 PID。
 *
 * 全局模式：跟踪 map 坐标系固定目标；可配置是否控制 yaw。
 * 视觉模式：跟踪相机给出的 base_link 相对位移，只控制前后、左右，yaw_rate 固定为 0。
 *
 * /pid/cmd_vel 始终用 map frame 发布，以兼容现有 flight_controller_bridge。视觉模式
 * 会先把机体系速度旋转到 map，桥接层再将其逆旋转回机体系后发给飞控。
 */
class PositionPidNode : public rclcpp::Node
{
public:
  PositionPidNode()
  : Node("position_pid_controller")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    pose_timeout_ms_ = declare_parameter<int>("pose_timeout_ms", 500);
    valid_timeout_ms_ = declare_parameter<int>("valid_timeout_ms", 500);
    vision_timeout_ms_ = declare_parameter<int>("vision_timeout_ms", 500);
    print_status_ = declare_parameter<bool>("print_status", true);
    enable_global_yaw_control_ =
      declare_parameter<bool>("enable_global_yaw_control", true);

    target_x_ = declare_parameter<double>("target_x", 0.0);
    target_y_ = declare_parameter<double>("target_y", 0.0);
    target_yaw_ = wrap_angle(declare_parameter<double>("target_yaw", 0.0));

    const double derivative_filter_alpha =
      declare_parameter<double>("derivative_filter_alpha", 0.8);

    const double x_kp = declare_parameter<double>("x_kp", 0.5);
    const double x_ki = declare_parameter<double>("x_ki", 0.01);
    const double x_kd = declare_parameter<double>("x_kd", 0.1);
    const double x_integral_limit = declare_parameter<double>("x_integral_limit", 1.0);
    const double x_output_limit = declare_parameter<double>("x_output_limit", 1.0);

    const double y_kp = declare_parameter<double>("y_kp", 0.5);
    const double y_ki = declare_parameter<double>("y_ki", 0.01);
    const double y_kd = declare_parameter<double>("y_kd", 0.1);
    const double y_integral_limit = declare_parameter<double>("y_integral_limit", 1.0);
    const double y_output_limit = declare_parameter<double>("y_output_limit", 1.0);

    const double yaw_kp = declare_parameter<double>("yaw_kp", 1.0);
    const double yaw_ki = declare_parameter<double>("yaw_ki", 0.01);
    const double yaw_kd = declare_parameter<double>("yaw_kd", 0.1);
    const double yaw_integral_limit =
      declare_parameter<double>("yaw_integral_limit", 1.0);
    const double yaw_output_limit = declare_parameter<double>("yaw_output_limit", 0.5);

    min_xy_speed_ = declare_parameter<double>("min_xy_speed", 0.08);
    max_xy_speed_ = declare_parameter<double>("max_xy_speed", 0.25);
    right_shift_max_xy_speed_ =
      declare_parameter<double>("right_shift_max_xy_speed", 0.19);
    forward_chase_max_xy_speed_ =
      declare_parameter<double>("forward_chase_max_xy_speed", 0.23);
    task2_max_xy_speed_ = declare_parameter<double>("task2_max_xy_speed", 0.28);
    task2_right_shift_max_xy_speed_ =
      declare_parameter<double>("task2_right_shift_max_xy_speed", 0.23);
    task2_forward_chase_max_xy_speed_ =
      declare_parameter<double>("task2_forward_chase_max_xy_speed", 0.25);
    min_yaw_rate_ = declare_parameter<double>("min_yaw_rate", 0.10);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.50);

    const double position_enter_tolerance =
      declare_parameter<double>("position_enter_tolerance", 0.10);
    const double position_exit_tolerance =
      declare_parameter<double>("position_exit_tolerance", 0.15);
    const double takeoff_hover_x_enter_tolerance =
      declare_parameter<double>("takeoff_hover_x_enter_tolerance", 0.30);
    const double takeoff_hover_x_exit_tolerance =
      declare_parameter<double>("takeoff_hover_x_exit_tolerance", 0.35);
    const double takeoff_hover_y_enter_tolerance =
      declare_parameter<double>("takeoff_hover_y_enter_tolerance", 0.10);
    const double takeoff_hover_y_exit_tolerance =
      declare_parameter<double>("takeoff_hover_y_exit_tolerance", 0.15);
    const double yaw_enter_tolerance =
      declare_parameter<double>("yaw_enter_tolerance", 0.10);
    const double yaw_exit_tolerance =
      declare_parameter<double>("yaw_exit_tolerance", 0.15);

    const double vision_forward_kp =
      declare_parameter<double>("vision_forward_kp", 0.8);
    const double vision_forward_ki =
      declare_parameter<double>("vision_forward_ki", 0.0);
    const double vision_forward_kd =
      declare_parameter<double>("vision_forward_kd", 0.05);
    const double vision_left_kp =
      declare_parameter<double>("vision_left_kp", 0.8);
    const double vision_left_ki =
      declare_parameter<double>("vision_left_ki", 0.0);
    const double vision_left_kd =
      declare_parameter<double>("vision_left_kd", 0.05);
    const double vision_integral_limit =
      declare_parameter<double>("vision_integral_limit", 0.5);
    const double vision_axis_output_limit =
      declare_parameter<double>("vision_axis_output_limit", 0.5);
    const double vision_derivative_filter_alpha =
      declare_parameter<double>("vision_derivative_filter_alpha", 0.8);
    vision_min_xy_speed_ =
      declare_parameter<double>("vision_min_xy_speed", 0.05);
    vision_max_xy_speed_ =
      declare_parameter<double>("vision_max_xy_speed", 0.20);
    const double vision_enter_tolerance =
      declare_parameter<double>("vision_enter_tolerance", 0.05);
    const double vision_exit_tolerance =
      declare_parameter<double>("vision_exit_tolerance", 0.08);

    validate_node_parameters();

    x_pid_ = std::make_unique<PidAxis>(
      x_kp, x_ki, x_kd, x_integral_limit, x_output_limit,
      derivative_filter_alpha);
    y_pid_ = std::make_unique<PidAxis>(
      y_kp, y_ki, y_kd, y_integral_limit, y_output_limit,
      derivative_filter_alpha);
    yaw_pid_ = std::make_unique<PidAxis>(
      yaw_kp, yaw_ki, yaw_kd, yaw_integral_limit, yaw_output_limit,
      derivative_filter_alpha);
    vision_forward_pid_ = std::make_unique<PidAxis>(
      vision_forward_kp, vision_forward_ki, vision_forward_kd,
      vision_integral_limit, vision_axis_output_limit,
      vision_derivative_filter_alpha);
    vision_left_pid_ = std::make_unique<PidAxis>(
      vision_left_kp, vision_left_ki, vision_left_kd,
      vision_integral_limit, vision_axis_output_limit,
      vision_derivative_filter_alpha);

    position_hold_ = std::make_unique<HysteresisHold>(
      position_enter_tolerance, position_exit_tolerance);
    takeoff_hover_x_hold_ = std::make_unique<HysteresisHold>(
      takeoff_hover_x_enter_tolerance, takeoff_hover_x_exit_tolerance);
    takeoff_hover_y_hold_ = std::make_unique<HysteresisHold>(
      takeoff_hover_y_enter_tolerance, takeoff_hover_y_exit_tolerance);
    yaw_hold_ = std::make_unique<HysteresisHold>(
      yaw_enter_tolerance, yaw_exit_tolerance);
    vision_position_hold_ = std::make_unique<HysteresisHold>(
      vision_enter_tolerance, vision_exit_tolerance);

    cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>("/pid/cmd_vel", 10);
    control_valid_publisher_ =
      create_publisher<std_msgs::msg::Bool>("/pid/control_valid", 10);
    target_reached_publisher_ =
      create_publisher<std_msgs::msg::Bool>("/pid/target_reached", 10);
    visual_control_active_publisher_ =
      create_publisher<std_msgs::msg::Bool>("/pid/visual_control_active", 10);

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/map/current_pose", reliable_qos,
      std::bind(&PositionPidNode::pose_callback, this, std::placeholders::_1));
    pose_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/map/current_pose_valid", reliable_qos,
      std::bind(&PositionPidNode::pose_valid_callback, this, std::placeholders::_1));
    target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pid/target_pose", reliable_qos,
      std::bind(&PositionPidNode::target_callback, this, std::placeholders::_1));
    phase_subscription_ = create_subscription<std_msgs::msg::String>(
      "/competition/phase", reliable_qos,
      std::bind(&PositionPidNode::phase_callback, this, std::placeholders::_1));
    active_task_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "/competition/active_task_mode", reliable_qos,
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        active_task_mode_ = message->data;
      });
    vision_request_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/competition/vision_assist_requested", reliable_qos,
      std::bind(&PositionPidNode::vision_request_callback, this, std::placeholders::_1));
    vision_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/vision/target_valid", reliable_qos,
      std::bind(&PositionPidNode::vision_valid_callback, this, std::placeholders::_1));
    vision_error_subscription_ =
      create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/vision/relative_error", reliable_qos,
      std::bind(&PositionPidNode::vision_error_callback, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
    control_timer_ = create_wall_timer(
      timer_period, std::bind(&PositionPidNode::control_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "双模式 PID 已启动：全局目标=(%.3f, %.3f, %.3f rad)，全局yaw控制=%s，"
      "视觉坐标=%s，频率=%.1f Hz；task1全局/右移/前追="
      "%.2f/%.2f/%.2f，task2=%.2f/%.2f/%.2f m/s",
      target_x_, target_y_, target_yaw_,
      enable_global_yaw_control_ ? "启用" : "禁用",
      body_frame_.c_str(), control_rate_hz_, max_xy_speed_,
      right_shift_max_xy_speed_, forward_chase_max_xy_speed_,
      task2_max_xy_speed_, task2_right_shift_max_xy_speed_,
      task2_forward_chase_max_xy_speed_);
    RCLCPP_WARN(
      get_logger(),
      "视觉接口约定：relative_error.x 向前为正、y 向左为正、单位米；视觉模式 yaw_rate=0。");
  }

private:
  void validate_node_parameters() const
  {
    if (map_frame_.empty() || body_frame_.empty()) {
      throw std::invalid_argument("map_frame 和 body_frame 不能为空");
    }
    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
      throw std::invalid_argument("control_rate_hz 必须大于 0");
    }
    if (pose_timeout_ms_ <= 0 || valid_timeout_ms_ <= 0 || vision_timeout_ms_ <= 0) {
      throw std::invalid_argument("所有输入超时参数必须大于 0");
    }
    if (!std::isfinite(target_x_) || !std::isfinite(target_y_) ||
      !std::isfinite(target_yaw_))
    {
      throw std::invalid_argument("YAML 初始目标必须是有限数值");
    }
    validate_speed_limits(min_xy_speed_, max_xy_speed_);
    validate_speed_limits(min_xy_speed_, right_shift_max_xy_speed_);
    validate_speed_limits(min_xy_speed_, forward_chase_max_xy_speed_);
    validate_speed_limits(min_xy_speed_, task2_max_xy_speed_);
    validate_speed_limits(min_xy_speed_, task2_right_shift_max_xy_speed_);
    validate_speed_limits(min_xy_speed_, task2_forward_chase_max_xy_speed_);
    if (right_shift_max_xy_speed_ > max_xy_speed_ ||
      forward_chase_max_xy_speed_ > max_xy_speed_)
    {
      throw std::invalid_argument("右移和前追速度上限不能超过全局平面速度上限");
    }
    if (task2_right_shift_max_xy_speed_ > task2_max_xy_speed_ ||
      task2_forward_chase_max_xy_speed_ > task2_max_xy_speed_)
    {
      throw std::invalid_argument("task2右移和前追速度上限不能超过task2全局平面速度上限");
    }
    validate_speed_limits(min_yaw_rate_, max_yaw_rate_);
    validate_speed_limits(vision_min_xy_speed_, vision_max_xy_speed_);
  }

  static bool quaternion_to_yaw(
    const geometry_msgs::msg::Quaternion & orientation, double & yaw)
  {
    if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      return false;
    }

    tf2::Quaternion quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);
    const double norm_squared = quaternion.length2();
    if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12) {
      return false;
    }
    quaternion.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;
    return std::isfinite(yaw);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      has_pose_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拒绝当前位姿：frame_id 应为 '%s'，实际为 '%s'",
        map_frame_.c_str(), message->header.frame_id.c_str());
      return;
    }

    const auto & position = message->pose.position;
    double yaw = 0.0;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !quaternion_to_yaw(message->pose.orientation, yaw))
    {
      has_pose_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拒绝当前位姿：x/y 或姿态四元数非法");
      return;
    }

    current_x_ = position.x;
    current_y_ = position.y;
    current_yaw_ = wrap_angle(yaw);
    current_pose_stamp_ = rclcpp::Time(
      message->header.stamp, get_clock()->get_clock_type());
    has_pose_ = true;
  }

  void pose_valid_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    pose_valid_ = message->data;
    has_pose_valid_message_ = true;
    pose_valid_received_time_ = now();
  }

  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "拒绝目标：frame_id 应为 '%s'，实际为 '%s'",
        map_frame_.c_str(), message->header.frame_id.c_str());
      return;
    }

    const auto & position = message->pose.position;
    double yaw = 0.0;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !quaternion_to_yaw(message->pose.orientation, yaw))
    {
      RCLCPP_WARN(get_logger(), "拒绝目标：x/y 或姿态四元数非法");
      return;
    }

    target_x_ = position.x;
    target_y_ = position.y;
    target_yaw_ = wrap_angle(yaw);
    reset_controller_state();
    RCLCPP_INFO(
      get_logger(), "收到全局目标：x=%.3f m, y=%.3f m, yaw=%.3f rad",
      target_x_, target_y_, target_yaw_);
  }

  void phase_callback(const std_msgs::msg::String::SharedPtr message)
  {
    if (mission_phase_ == message->data) {
      return;
    }
    mission_phase_ = message->data;
    reset_controller_state();
    RCLCPP_INFO(get_logger(), "PID收到任务阶段：%s", mission_phase_.c_str());
  }

  void vision_request_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    const bool changed =
      !has_vision_request_message_ || vision_assist_requested_ != message->data;
    vision_assist_requested_ = message->data;
    has_vision_request_message_ = true;
    if (changed) {
      RCLCPP_INFO(
        get_logger(), "视觉相对控制请求变为 %s",
        vision_assist_requested_ ? "true" : "false");
    }
  }

  void vision_valid_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    vision_target_valid_ = message->data;
    has_vision_valid_message_ = true;
  }

  void vision_error_callback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
  {
    if (message->header.frame_id != body_frame_) {
      has_vision_error_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拒绝视觉相对位移：frame_id 应为 '%s'，实际为 '%s'",
        body_frame_.c_str(), message->header.frame_id.c_str());
      return;
    }
    if (!std::isfinite(message->vector.x) || !std::isfinite(message->vector.y)) {
      has_vision_error_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "拒绝视觉相对位移：x/y 不是有限数值");
      return;
    }

    vision_error_forward_ = message->vector.x;
    vision_error_left_ = message->vector.y;
    // 用本机接收时间判断新鲜度，不要求相机节点与本机时钟严格同步。
    vision_error_received_time_ = now();
    has_vision_error_ = true;
  }

  static bool invalid_age(
    const rclcpp::Time & current_time,
    const rclcpp::Time & received_time,
    int timeout_ms)
  {
    const double age_ms = (current_time - received_time).seconds() * 1000.0;
    return !std::isfinite(age_ms) || age_ms < 0.0 ||
           age_ms > static_cast<double>(timeout_ms);
  }

  std::string common_invalid_reason(const rclcpp::Time & current_time) const
  {
    if (!has_pose_) {
      return "尚未收到合法的 /map/current_pose";
    }
    if (!has_pose_valid_message_) {
      return "尚未收到 /map/current_pose_valid";
    }
    if (!pose_valid_) {
      return "地图坐标提供者报告 valid=false";
    }
    if (current_pose_stamp_.nanoseconds() <= 0) {
      return "当前位姿的 header.stamp 为空";
    }
    if (invalid_age(current_time, current_pose_stamp_, pose_timeout_ms_)) {
      return "当前位姿时间戳异常或已经超时";
    }
    if (invalid_age(current_time, pose_valid_received_time_, valid_timeout_ms_)) {
      return "当前位姿有效标志计时异常或已经超时";
    }
    return "";
  }

  std::string visual_invalid_reason(const rclcpp::Time & current_time) const
  {
    if (!has_vision_valid_message_) {
      return "尚未收到 /vision/target_valid";
    }
    if (!vision_target_valid_) {
      return "视觉节点报告 target_valid=false";
    }
    if (!has_vision_error_) {
      return "尚未收到合法的 /vision/relative_error";
    }
    if (invalid_age(current_time, vision_error_received_time_, vision_timeout_ms_)) {
      return "视觉相对位移已经超时";
    }
    return "";
  }

  bool update_control_dt(const rclcpp::Time & current_time, double & dt_sec)
  {
    const double nominal_dt = 1.0 / control_rate_hz_;
    const double maximum_dt = std::max(0.5, 5.0 * nominal_dt);
    dt_sec = nominal_dt;
    if (has_last_control_time_) {
      dt_sec = (current_time - last_control_time_).seconds();
      if (!std::isfinite(dt_sec) || dt_sec <= 0.0 || dt_sec > maximum_dt) {
        reset_controller_state();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "控制计时异常（dt=%.6f s），本周期输出零速度", dt_sec);
        return false;
      }
    }
    last_control_time_ = current_time;
    has_last_control_time_ = true;
    return true;
  }

  void run_global_control(const rclcpp::Time & current_time, double dt_sec)
  {
    const double error_x = target_x_ - current_x_;
    const double error_y = target_y_ - current_y_;
    const double position_error = std::hypot(error_x, error_y);
    const double yaw_error = wrap_angle(target_yaw_ - current_yaw_);

    const bool takeoff_hover = mission_phase_ == "TAKEOFF_HOVER";
    bool x_holding = false;
    bool y_holding = false;
    bool position_holding = false;
    if (takeoff_hover) {
      position_hold_->reset();
      x_holding = takeoff_hover_x_hold_->update(std::abs(error_x));
      y_holding = takeoff_hover_y_hold_->update(std::abs(error_y));
      position_holding = x_holding && y_holding;
    } else {
      takeoff_hover_x_hold_->reset();
      takeoff_hover_y_hold_->reset();
      position_holding = position_hold_->update(position_error);
    }
    bool yaw_holding = true;
    if (enable_global_yaw_control_) {
      yaw_holding = yaw_hold_->update(std::abs(yaw_error));
    } else {
      yaw_pid_->reset();
      yaw_hold_->reset();
    }

    double command_x = 0.0;
    double command_y = 0.0;
    if (position_holding) {
      x_pid_->reset();
      y_pid_->reset();
    } else {
      double raw_x = 0.0;
      double raw_y = 0.0;
      if (!takeoff_hover || !x_holding) {
        raw_x = x_pid_->update_error(error_x, dt_sec);
      } else {
        x_pid_->reset();
      }
      if (!takeoff_hover || !y_holding) {
        raw_y = y_pid_->update_error(error_y, dt_sec);
      } else {
        y_pid_->reset();
      }
      const double phase_max_xy_speed = active_global_max_xy_speed();
      const PlanarCommand limited = apply_planar_speed_limits(
        raw_x, raw_y, error_x, error_y, min_xy_speed_, phase_max_xy_speed);
      command_x = limited.x;
      command_y = limited.y;
      if (takeoff_hover && x_holding) {
        command_x = 0.0;
      }
      if (takeoff_hover && y_holding) {
        command_y = 0.0;
      }
    }

    double command_yaw_rate = 0.0;
    if (enable_global_yaw_control_) {
      if (yaw_holding) {
        yaw_pid_->reset();
      } else {
        const double raw_yaw_rate = yaw_pid_->update_error(yaw_error, dt_sec);
        command_yaw_rate = apply_yaw_rate_limits(
          raw_yaw_rate, yaw_error, min_yaw_rate_, max_yaw_rate_);
      }
    }

    const bool target_reached = position_holding && yaw_holding;
    publish_output(
      current_time, command_x, command_y, command_yaw_rate,
      true, target_reached, false);

    if (print_status_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[全局] 位置误差=%.3f m (x=%.3f, y=%.3f), yaw误差=%.3f rad；map速度=(%.3f, %.3f), "
        "yaw_rate=%.3f；到达=%s",
        position_error, error_x, error_y, yaw_error, command_x, command_y, command_yaw_rate,
        target_reached ? "true" : "false");
    }
  }

  double active_global_max_xy_speed() const
  {
    const bool task2_active = active_task_mode_ == 2U;
    if (mission_phase_ == "RIGHT_SHIFT") {
      return task2_active ? task2_right_shift_max_xy_speed_ : right_shift_max_xy_speed_;
    }
    if (mission_phase_ == "FORWARD_CHASE") {
      return task2_active ? task2_forward_chase_max_xy_speed_ : forward_chase_max_xy_speed_;
    }
    return task2_active ? task2_max_xy_speed_ : max_xy_speed_;
  }

  void run_visual_control(const rclcpp::Time & current_time, double dt_sec)
  {
    const double position_error =
      std::hypot(vision_error_forward_, vision_error_left_);
    const bool position_holding = vision_position_hold_->update(position_error);

    double command_forward = 0.0;
    double command_left = 0.0;
    if (position_holding) {
      vision_forward_pid_->reset();
      vision_left_pid_->reset();
    } else {
      const double raw_forward =
        vision_forward_pid_->update_error(vision_error_forward_, dt_sec);
      const double raw_left =
        vision_left_pid_->update_error(vision_error_left_, dt_sec);
      const PlanarCommand limited = apply_planar_speed_limits(
        raw_forward, raw_left,
        vision_error_forward_, vision_error_left_,
        vision_min_xy_speed_, vision_max_xy_speed_);
      command_forward = limited.x;
      command_left = limited.y;
    }

    const PlanarCommand map_command =
      body_command_to_map(command_forward, command_left, current_yaw_);

    // 视觉阶段不改变机头方向。飞控的定点模式负责保持当前 yaw。
    constexpr double command_yaw_rate = 0.0;
    publish_output(
      current_time, map_command.x, map_command.y, command_yaw_rate,
      true, position_holding, true);

    if (print_status_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[视觉] 相对误差：前=%.3f m, 左=%.3f m；机体系速度=(%.3f, %.3f)；"
        "yaw_rate=0；对准=%s",
        vision_error_forward_, vision_error_left_,
        command_forward, command_left,
        position_holding ? "true" : "false");
    }
  }

  void control_timer_callback()
  {
    const rclcpp::Time current_time = now();
    const bool visual_requested =
      has_vision_request_message_ && vision_assist_requested_;

    if (visual_requested != visual_mode_selected_) {
      visual_mode_selected_ = visual_requested;
      reset_controller_state();
      RCLCPP_INFO(
        get_logger(), "PID 已切换到%s模式",
        visual_mode_selected_ ? "视觉相对位移" : "地图固定目标");
    }

    std::string reason = common_invalid_reason(current_time);
    if (reason.empty() && visual_requested) {
      reason = visual_invalid_reason(current_time);
    }
    if (!reason.empty()) {
      reset_controller_state();
      publish_output(current_time, 0.0, 0.0, 0.0, false, false, false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "控制无效并输出零速度：%s", reason.c_str());
      return;
    }

    double dt_sec = 0.0;
    if (!update_control_dt(current_time, dt_sec)) {
      publish_output(current_time, 0.0, 0.0, 0.0, false, false, false);
      return;
    }

    if (visual_requested) {
      run_visual_control(current_time, dt_sec);
    } else {
      run_global_control(current_time, dt_sec);
    }
  }

  void publish_output(
    const rclcpp::Time & stamp,
    double velocity_x, double velocity_y, double yaw_rate,
    bool control_valid, bool target_reached, bool visual_control_active)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = stamp;
    command.header.frame_id = map_frame_;
    command.twist.linear.x = velocity_x;
    command.twist.linear.y = velocity_y;
    command.twist.angular.z = yaw_rate;
    cmd_vel_publisher_->publish(command);

    std_msgs::msg::Bool valid_message;
    valid_message.data = control_valid;
    control_valid_publisher_->publish(valid_message);

    std_msgs::msg::Bool reached_message;
    reached_message.data = target_reached;
    target_reached_publisher_->publish(reached_message);

    std_msgs::msg::Bool visual_message;
    visual_message.data = visual_control_active;
    visual_control_active_publisher_->publish(visual_message);
  }

  void reset_controller_state()
  {
    x_pid_->reset();
    y_pid_->reset();
    yaw_pid_->reset();
    vision_forward_pid_->reset();
    vision_left_pid_->reset();
    position_hold_->reset();
    takeoff_hover_x_hold_->reset();
    takeoff_hover_y_hold_->reset();
    yaw_hold_->reset();
    vision_position_hold_->reset();
    has_last_control_time_ = false;
  }

  std::string map_frame_;
  std::string body_frame_;
  double control_rate_hz_{20.0};
  int pose_timeout_ms_{500};
  int valid_timeout_ms_{500};
  int vision_timeout_ms_{500};
  bool print_status_{true};
  bool enable_global_yaw_control_{true};
  std::string mission_phase_{"WAIT_START"};
  std::uint8_t active_task_mode_{0U};

  double target_x_{0.0};
  double target_y_{0.0};
  double target_yaw_{0.0};
  double min_xy_speed_{0.08};
  double max_xy_speed_{0.25};
  double right_shift_max_xy_speed_{0.19};
  double forward_chase_max_xy_speed_{0.23};
  double task2_max_xy_speed_{0.28};
  double task2_right_shift_max_xy_speed_{0.23};
  double task2_forward_chase_max_xy_speed_{0.25};
  double min_yaw_rate_{0.10};
  double max_yaw_rate_{0.50};
  double vision_min_xy_speed_{0.05};
  double vision_max_xy_speed_{0.20};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  bool has_pose_{false};
  bool has_pose_valid_message_{false};
  bool pose_valid_{false};
  bool has_last_control_time_{false};
  rclcpp::Time current_pose_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pose_valid_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};

  bool has_vision_request_message_{false};
  bool vision_assist_requested_{false};
  bool visual_mode_selected_{false};
  bool has_vision_valid_message_{false};
  bool vision_target_valid_{false};
  bool has_vision_error_{false};
  double vision_error_forward_{0.0};
  double vision_error_left_{0.0};
  rclcpp::Time vision_error_received_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<PidAxis> x_pid_;
  std::unique_ptr<PidAxis> y_pid_;
  std::unique_ptr<PidAxis> yaw_pid_;
  std::unique_ptr<PidAxis> vision_forward_pid_;
  std::unique_ptr<PidAxis> vision_left_pid_;
  std::unique_ptr<HysteresisHold> position_hold_;
  std::unique_ptr<HysteresisHold> takeoff_hover_x_hold_;
  std::unique_ptr<HysteresisHold> takeoff_hover_y_hold_;
  std::unique_ptr<HysteresisHold> yaw_hold_;
  std::unique_ptr<HysteresisHold> vision_position_hold_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_valid_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_reached_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_control_active_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pose_valid_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr active_task_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_valid_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    vision_error_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace position_pid_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<position_pid_controller::PositionPidNode>());
  rclcpp::shutdown();
  return 0;
}
