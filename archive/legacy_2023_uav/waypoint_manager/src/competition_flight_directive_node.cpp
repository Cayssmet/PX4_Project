#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uav_interfaces/msg/flight_directive.hpp"
#include "uav_interfaces/msg/flight_status.hpp"
#include "uav_interfaces/msg/waypoint.hpp"

namespace waypoint_manager
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

class CompetitionFlightDirectiveNode : public rclcpp::Node
{
public:
  CompetitionFlightDirectiveNode()
  : Node("competition_flight_directive")
  {
    read_parameters();
    validate_parameters();
    flight_session_id_ = make_flight_session_id();

    const auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable().transient_local();
    const auto normal_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    directive_publisher_ = create_publisher<uav_interfaces::msg::FlightDirective>(
      "/flight/directive", retained_qos);
    phase_subscription_ = create_subscription<std_msgs::msg::String>(
      "/competition/phase", retained_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {handle_phase_update(message->data);});
    waypoint_subscription_ = create_subscription<uav_interfaces::msg::Waypoint>(
      "/waypoint/current", retained_qos,
      [this](const uav_interfaces::msg::Waypoint::SharedPtr message) {
        waypoint_ = *message;
        waypoint_received_ = true;
      });
    status_subscription_ = create_subscription<uav_interfaces::msg::FlightStatus>(
      "/flight/status", normal_qos,
      [this](const uav_interfaces::msg::FlightStatus::SharedPtr message) {
        status_ = *message;
        status_received_ = true;
        status_time_ = SteadyClock::now();
        if (phase_ == "TASK2_DESCEND" && status_.height_valid &&
          std::isfinite(status_.current_height) &&
          status_.current_height >= 0.1F &&
          status_.current_height <= static_cast<float>(max_target_height_m_))
        {
          task2_last_valid_descent_height_ = status_.current_height;
          task2_last_valid_descent_height_available_ = true;
        }
      });

    const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&CompetitionFlightDirectiveNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "V2任务指令节点已启动：flight_session=%u (0x%08X)，等待STANDBY回显后才允许TAKEOFF",
      flight_session_id_, flight_session_id_);
  }

private:
  void read_parameters()
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    status_timeout_ms_ = declare_parameter<int>("status_timeout_ms", 500);
    min_takeoff_height_m_ = declare_parameter<double>("min_takeoff_height_m", 0.3);
    max_target_height_m_ = declare_parameter<double>("max_target_height_m", 1.7);
    flight_session_id_override_parameter_ =
      declare_parameter<int64_t>("flight_session_id_override", 0);
    takeoff_action_id_parameter_ = declare_parameter<int64_t>("takeoff_action_id", 1);
    final_land_action_id_parameter_ = declare_parameter<int64_t>("final_land_action_id", 2);
    task2_descent_velocity_mps_ =
      declare_parameter<double>("task2_descent_velocity_mps", -0.12);
    task2_final_land_trigger_height_m_ =
      declare_parameter<double>("task2_final_land_trigger_height_m", 0.20);
    print_status_ = declare_parameter<bool>("print_status", true);
  }

  static std::uint32_t checked_u32(const std::int64_t value, const std::string & name)
  {
    if (value < 0 ||
      value > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()))
    {
      throw std::invalid_argument(name + "超出uint32范围");
    }
    return static_cast<std::uint32_t>(value);
  }

  void validate_parameters()
  {
    if (!std::isfinite(publish_rate_hz_) ||
      publish_rate_hz_ < 10.0 || publish_rate_hz_ > 50.0)
    {
      throw std::invalid_argument("publish_rate_hz必须位于[10,50] Hz");
    }
    if (status_timeout_ms_ <= 0) {
      throw std::invalid_argument("status_timeout_ms必须大于0");
    }
    if (!std::isfinite(min_takeoff_height_m_) ||
      !std::isfinite(max_target_height_m_) ||
      min_takeoff_height_m_ < 0.3 ||
      max_target_height_m_ < min_takeoff_height_m_ ||
      max_target_height_m_ > 1.7)
    {
      throw std::invalid_argument("V2高度安全范围非法");
    }

    if (!std::isfinite(task2_descent_velocity_mps_) ||
      task2_descent_velocity_mps_ < -0.5 || task2_descent_velocity_mps_ >= 0.0)
    {
      throw std::invalid_argument("task2_descent_velocity_mps必须位于[-0.5,0) m/s");
    }
    if (!std::isfinite(task2_final_land_trigger_height_m_) ||
      task2_final_land_trigger_height_m_ < 0.1 ||
      task2_final_land_trigger_height_m_ > max_target_height_m_)
    {
      throw std::invalid_argument("task2_final_land_trigger_height_m超出安全范围");
    }

    flight_session_id_override_ = checked_u32(
      flight_session_id_override_parameter_, "flight_session_id_override");
    if (flight_session_id_override_ != 0U && flight_session_id_override_ <= 2U) {
      throw std::invalid_argument("flight_session_id_override必须为0或大于2");
    }
    takeoff_action_id_ = checked_u32(takeoff_action_id_parameter_, "takeoff_action_id");
    final_land_action_id_ = checked_u32(
      final_land_action_id_parameter_, "final_land_action_id");
    if (takeoff_action_id_ == 0U || final_land_action_id_ <= takeoff_action_id_) {
      throw std::invalid_argument("TAKEOFF action必须非零，FINAL_LAND action必须更大");
    }
  }

  std::uint32_t make_flight_session_id() const
  {
    if (flight_session_id_override_ != 0U) {
      return flight_session_id_override_;
    }

    std::uint64_t mixed = static_cast<std::uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count());
    mixed ^= static_cast<std::uint64_t>(
      SteadyClock::now().time_since_epoch().count());
    mixed ^= static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(this));
    mixed ^= mixed >> 33U;
    mixed *= 0xff51afd7ed558ccdULL;
    mixed ^= mixed >> 33U;
    mixed *= 0xc4ceb9fe1a85ec53ULL;
    mixed ^= mixed >> 33U;
    std::uint32_t session = static_cast<std::uint32_t>(mixed ^ (mixed >> 32U));
    if (session <= 2U) {
      session += 3U;
    }
    return session;
  }

  void handle_phase_update(const std::string & new_phase)
  {
    if (phase_ == "TASK2_DESCEND" && new_phase == "TASK2_ALIGN_LAND") {
      float hold_height = 0.0F;
      if (status_received_ && status_.height_valid &&
        std::isfinite(status_.current_height) &&
        status_.current_height >= 0.1F &&
        status_.current_height <= static_cast<float>(max_target_height_m_))
      {
        hold_height = status_.current_height;
      } else if (task2_last_valid_descent_height_available_) {
        hold_height = task2_last_valid_descent_height_;
      }

      task2_descent_hold_active_ = true;
      task2_descent_hold_height_ = hold_height;
      if (valid_target_height(task2_descent_hold_height_, false)) {
        RCLCPP_WARN(
          get_logger(),
          "task2下降中视觉丢失：锁存当前高度%.3f m等待重新对准，不返回巡航高度",
          task2_descent_hold_height_);
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "task2下降中视觉丢失但没有有效当前高度：撤销有效控制，禁止回升至巡航高度");
      }
    }

    if (new_phase == "WAIT_START" || new_phase == "WAIT_READY" ||
      new_phase == "TAKEOFF_HOVER" || new_phase == "TASK2_COMPLETE")
    {
      task2_descent_hold_active_ = false;
      task2_descent_hold_height_ = 0.0F;
      task2_last_valid_descent_height_available_ = false;
    }

    phase_ = new_phase;
    phase_received_ = true;
  }

  bool status_is_fresh(const SteadyTime & current_time) const
  {
    return status_received_ &&
      current_time - status_time_ <= std::chrono::milliseconds(status_timeout_ms_);
  }

  bool matching_v2_status(const SteadyTime & current_time) const
  {
    return status_is_fresh(current_time) &&
      status_.protocol_version == uav_interfaces::msg::FlightStatus::PROTOCOL_V2 &&
      status_.communication_ok &&
      status_.active_session_id == flight_session_id_;
  }

  bool control_status_is_safe(const SteadyTime & current_time) const
  {
    return matching_v2_status(current_time) && !status_.failsafe &&
      status_.error_code == 0U;
  }

  void update_session_and_action_state(const SteadyTime & current_time)
  {
    if (matching_v2_status(current_time) && !status_.failsafe &&
      status_.error_code == 0U)
    {
      if (!session_confirmed_) {
        RCLCPP_INFO(
          get_logger(), "飞控已回显flight_session=0x%08X，允许后续V2动作",
          flight_session_id_);
      }
      session_confirmed_ = true;
    } else if (session_confirmed_ && status_is_fresh(current_time) &&
      status_.protocol_version == uav_interfaces::msg::FlightStatus::PROTOCOL_V2 &&
      status_.active_session_id != flight_session_id_ &&
      !status_.armed && !status_.airborne)
    {
      session_confirmed_ = false;
      takeoff_succeeded_latched_ = false;
      task2_final_land_started_ = false;
      action_fault_ = false;
      RCLCPP_WARN(get_logger(), "飞控session丢失，退回STANDBY重新握手");
    }

    const bool matching_takeoff_action =
      matching_v2_status(current_time) &&
      status_.active_action_id == takeoff_action_id_;
    if (!takeoff_succeeded_latched_ && matching_takeoff_action &&
      status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_SUCCEEDED &&
      !status_.failsafe && status_.error_code == 0U)
    {
      takeoff_succeeded_latched_ = true;
      RCLCPP_INFO(
        get_logger(),
        "已锁存当前flight session的TAKEOFF成功；后续控制不再读取HR");
    }
    if (matching_takeoff_action &&
      (status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_REJECTED ||
      status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_FAILED))
    {
      if (!action_fault_) {
        RCLCPP_ERROR(
          get_logger(), "TAKEOFF被飞控拒绝或执行失败：result=%u error=0x%02X",
          status_.action_result, status_.error_code);
      }
      action_fault_ = true;
    }

    const bool matching_final_land_action =
      matching_v2_status(current_time) && status_.active_action_id == final_land_action_id_;
    if (matching_final_land_action &&
      (status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_REJECTED ||
      status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_FAILED))
    {
      if (!action_fault_) {
        RCLCPP_ERROR(
          get_logger(), "FINAL_LAND被飞控拒绝或执行失败：result=%u error=0x%02X",
          status_.action_result, status_.error_code);
      }
      action_fault_ = true;
    }
  }

  void update_task2_final_land_latch(const SteadyTime & current_time)
  {
    const std::string phase = phase_received_ ? phase_ : "WAIT_START";
    if (phase == "WAIT_START" || phase == "WAIT_READY" || phase == "TAKEOFF_HOVER") {
      task2_final_land_started_ = false;
      return;
    }
    if (phase != "TASK2_DESCEND" || task2_final_land_started_) {
      return;
    }

    const bool final_land_already_active = matching_v2_status(current_time) &&
      status_.active_action_id == final_land_action_id_ &&
      (status_.current_mode == uav_interfaces::msg::FlightStatus::FINAL_LAND ||
      status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_PENDING ||
      status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_SUCCEEDED);
    const bool reached_handover_height = control_status_is_safe(current_time) &&
      status_.height_valid &&
      status_.current_height <= task2_final_land_trigger_height_m_ + 1.0e-3;
    if (final_land_already_active || reached_handover_height) {
      task2_final_land_started_ = true;
      RCLCPP_INFO(
        get_logger(),
        "task2下降达到%.2f m交接高度，锁存FINAL_LAND action=%u",
        task2_final_land_trigger_height_m_, final_land_action_id_);
    }
  }

  static bool phase_uses_height_hold(const std::string & phase)
  {
    return phase == "RIGHT_SHIFT" ||
           phase == "FORWARD_CHASE" ||
           phase == "TASK1_FOLLOW" ||
           phase == "TASK1_ALIGN_DROP" ||
           phase == "TASK1_DROP" ||
           phase == "TASK1_RETURN_HOME" ||
           phase == "TASK2_VISUAL_APPROACH" ||
           phase == "TASK2_ALIGN_LAND";
  }
  bool takeoff_action_succeeded(const SteadyTime & current_time) const
  {
    return takeoff_succeeded_latched_ && control_status_is_safe(current_time);
  }

  static bool phase_requests_final_land(const std::string & phase)
  {
    return phase == "TASK1_LAND_HOME";
  }

  static bool phase_disables_control(const std::string & phase)
  {
    return phase == "ERROR" || phase == "ABORTED" ||
           phase == "TASK1_COMPLETE" || phase == "TASK2_COMPLETE";
  }

  bool valid_target_height(const double value, const bool takeoff) const
  {
    const double minimum = takeoff ? min_takeoff_height_m_ : 0.1;
    return std::isfinite(value) && value >= minimum && value <= max_target_height_m_;
  }

  void latch_takeoff_height_if_ready()
  {
    const std::string phase = phase_received_ ? phase_ : "WAIT_START";
    if (phase != "TAKEOFF_HOVER" || takeoff_height_latched_ ||
      !waypoint_received_ || !valid_target_height(waypoint_.target_height, true))
    {
      return;
    }

    takeoff_target_height_ = static_cast<float>(waypoint_.target_height);
    takeoff_height_latched_ = true;
    RCLCPP_INFO(
      get_logger(),
      "已为当前flight session锁存TAKEOFF高度：%.3f m；action=%u后续重发不再跟随航点变化",
      takeoff_target_height_, takeoff_action_id_);
  }

  uav_interfaces::msg::FlightDirective make_directive(
    const SteadyTime & current_time, std::string & reason) const
  {
    uav_interfaces::msg::FlightDirective directive;
    directive.header.stamp = get_clock()->now();
    directive.control_mode = uav_interfaces::msg::FlightDirective::STANDBY;
    directive.session_id = flight_session_id_;
    directive.action_id = 0U;
    directive.vertical_velocity = 0.0F;
    directive.target_height = 0.0F;
    directive.control_valid = false;

    const std::string phase = phase_received_ ? phase_ : "WAIT_START";
    if (phase_disables_control(phase) || action_fault_) {
      reason = action_fault_ ? "飞控动作失败，撤销控制" :
        "任务结束或错误，撤销控制";
      return directive;
    }

    if (phase == "SAFE_HOLD") {
      // TAKEOFF 是不可被 STANDBY 取消的离散动作。若只是定位短暂失效、
      // 飞控本身仍健康且同一 action 正在执行，必须重发完全相同的 TAKEOFF；
      // 否则 STANDBY 会被飞控返回 ACTION_NOT_ALLOWED，反过来把恢复门锁死。
      const bool takeoff_pending = takeoff_height_latched_ &&
        !takeoff_succeeded_latched_ && matching_v2_status(current_time) &&
        status_.active_action_id == takeoff_action_id_ &&
        status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_PENDING &&
        control_status_is_safe(current_time);
      if (takeoff_pending) {
        directive.control_mode = uav_interfaces::msg::FlightDirective::TAKEOFF;
        directive.action_id = takeoff_action_id_;
        directive.target_height = takeoff_target_height_;
        directive.control_valid = true;
        reason = "TAKEOFF执行中SAFE_HOLD保持原动作，水平速度归零";
        return directive;
      }

      if (takeoff_succeeded_latched_ && takeoff_height_latched_ &&
        control_status_is_safe(current_time))
      {
        directive.control_mode = uav_interfaces::msg::FlightDirective::HEIGHT_HOLD;
        directive.target_height = takeoff_target_height_;
        directive.control_valid = true;
        reason = "空中SAFE_HOLD保持锁存高度，水平速度由飞控桥安全归零";
        return directive;
      }
      directive.control_valid = true;
      reason = "起飞前或飞控状态不安全，SAFE_HOLD发送零速STANDBY";
      return directive;
    }

    if (!session_confirmed_) {
      directive.control_valid = true;
      reason = "STANDBY建立flight session";
      return directive;
    }

    if (phase == "WAIT_START" || phase == "WAIT_READY") {
      directive.control_valid = true;
      reason = "任务未START，保持STANDBY";
      return directive;
    }

    if (!control_status_is_safe(current_time)) {
      reason = "V2状态过期、failsafe或错误，撤销有效控制";
      return directive;
    }

    if (phase == "TAKEOFF_HOVER") {
      if (!takeoff_height_latched_) {
        reason = "等待锁存合法起飞高度";
        return directive;
      }
      // V2 TAKEOFF 是离散的纯垂直动作，协议要求横向速度为零。
      // 动作成功后，任务全局阶段仍保持 TAKEOFF_HOVER，但改用连续的
      // HEIGHT_HOLD；这样 PID 才能把飞机保持在起飞点附近并完成悬停计时。
      if (takeoff_action_succeeded(current_time)) {
        directive.control_mode = uav_interfaces::msg::FlightDirective::HEIGHT_HOLD;
        directive.target_height = takeoff_target_height_;
        directive.control_valid = true;
        reason = "TAKEOFF成功后HEIGHT_HOLD水平PID";
        return directive;
      }
      directive.control_mode = uav_interfaces::msg::FlightDirective::TAKEOFF;
      directive.action_id = takeoff_action_id_;
      directive.target_height = takeoff_target_height_;
      directive.control_valid = true;
      reason = "TAKEOFF动作";
      return directive;
    }

    if (phase == "TASK2_ALIGN_LAND" && task2_descent_hold_active_) {
      if (!valid_target_height(task2_descent_hold_height_, false)) {
        reason = "task2下降中视觉丢失且当前高度无效，禁止返回巡航高度";
        return directive;
      }
      directive.control_mode = uav_interfaces::msg::FlightDirective::HEIGHT_HOLD;
      directive.target_height = task2_descent_hold_height_;
      directive.control_valid = true;
      reason = "task2视觉丢失，保持下降瞬间高度等待重新对准";
      return directive;
    }

    if (phase == "TASK2_DESCEND") {
      if (task2_final_land_started_) {
        directive.control_mode = uav_interfaces::msg::FlightDirective::FINAL_LAND;
        directive.action_id = final_land_action_id_;
        directive.control_valid = true;
        reason = "TASK2近地FINAL_LAND";
      } else {
        directive.control_mode = uav_interfaces::msg::FlightDirective::PRECISION_DESCENT;
        directive.vertical_velocity = static_cast<float>(task2_descent_velocity_mps_);
        directive.target_height = static_cast<float>(task2_final_land_trigger_height_m_);
        directive.control_valid = true;
        reason = "TASK2视觉对准精密下降";
      }
      return directive;
    }

    if (phase_uses_height_hold(phase)) {
      if (!waypoint_received_ ||
        !valid_target_height(waypoint_.target_height, false))
      {
        reason = "定高航点或高度无效";
        return directive;
      }
      directive.control_mode = uav_interfaces::msg::FlightDirective::HEIGHT_HOLD;
      directive.target_height = static_cast<float>(waypoint_.target_height);
      directive.control_valid = true;
      reason = "HEIGHT_HOLD水平PID";
      return directive;
    }

    if (phase_requests_final_land(phase)) {
      directive.control_mode = uav_interfaces::msg::FlightDirective::FINAL_LAND;
      directive.action_id = final_land_action_id_;
      directive.control_valid = true;
      reason = "FINAL_LAND动作";
      return directive;
    }

    directive.control_valid = true;
    reason = "未映射任务阶段，安全STANDBY";
    return directive;
  }

  void timer_callback()
  {
    const SteadyTime current_time = SteadyClock::now();
    update_session_and_action_state(current_time);
    latch_takeoff_height_if_ready();
    update_task2_final_land_latch(current_time);
    std::string reason;
    const auto directive = make_directive(current_time, reason);
    directive_publisher_->publish(directive);

    if (print_status_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "V2任务指令：phase=%s mode=%u session=0x%08X action=%u height=%.2f valid=%s；%s",
        phase_received_ ? phase_.c_str() : "WAIT_START",
        directive.control_mode, directive.session_id, directive.action_id,
        directive.target_height, directive.control_valid ? "true" : "false",
        reason.c_str());
    }
  }

  double publish_rate_hz_{20.0};
  int status_timeout_ms_{500};
  double min_takeoff_height_m_{0.3};
  double max_target_height_m_{1.7};
  double task2_descent_velocity_mps_{-0.12};
  double task2_final_land_trigger_height_m_{0.20};

  std::int64_t flight_session_id_override_parameter_{0};
  std::int64_t takeoff_action_id_parameter_{1};
  std::int64_t final_land_action_id_parameter_{2};
  std::uint32_t flight_session_id_override_{0U};
  std::uint32_t flight_session_id_{0U};
  std::uint32_t takeoff_action_id_{1U};
  std::uint32_t final_land_action_id_{2U};

  bool print_status_{true};
  bool phase_received_{false};
  bool waypoint_received_{false};
  bool status_received_{false};
  bool session_confirmed_{false};
  bool takeoff_height_latched_{false};
  bool takeoff_succeeded_latched_{false};
  bool task2_final_land_started_{false};
  bool task2_descent_hold_active_{false};
  bool task2_last_valid_descent_height_available_{false};
  float task2_descent_hold_height_{0.0F};
  float task2_last_valid_descent_height_{0.0F};
  float takeoff_target_height_{0.0F};
  bool action_fault_{false};
  std::string phase_{"WAIT_START"};
  SteadyTime status_time_{};
  uav_interfaces::msg::Waypoint waypoint_;
  uav_interfaces::msg::FlightStatus status_;

  rclcpp::Publisher<uav_interfaces::msg::FlightDirective>::SharedPtr directive_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::Waypoint>::SharedPtr waypoint_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::FlightStatus>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace waypoint_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<waypoint_manager::CompetitionFlightDirectiveNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("competition_flight_directive"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
