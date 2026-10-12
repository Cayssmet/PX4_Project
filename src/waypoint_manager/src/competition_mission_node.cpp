#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "uav_interfaces/msg/flight_status.hpp"
#include "uav_interfaces/msg/mission_status.hpp"
#include "uav_interfaces/msg/waypoint.hpp"

#include "waypoint_manager/arrival_tracker.hpp"
#include "waypoint_manager/competition_geometry.hpp"

namespace waypoint_manager
{

constexpr std::uint8_t kTaskPayloadDrop = 1U;
constexpr std::uint8_t kTaskDynamicLanding = 2U;

enum class CompetitionPhase : std::uint8_t
{
  WAIT_START,
  WAIT_READY,
  TAKEOFF_HOVER,
  LATERAL_SHIFT,
  FORWARD_CHASE,
  TASK1_FOLLOW,
  TASK1_ALIGN_DROP,
  TASK1_DROP,
  TASK1_RETURN_HOME,
  TASK1_LAND_HOME,
  TASK1_COMPLETE,
  TASK2_VISUAL_APPROACH,
  TASK2_ALIGN_LAND,
  TASK2_DESCEND,
  TASK2_ON_CAR,
  TASK2_HOLD_5S,
  TASK2_RETAKEOFF,
  TASK2_RETURN_HOME,
  TASK2_LAND_HOME,
  TASK2_COMPLETE,
  SAFE_HOLD,
  ERROR
};

class CompetitionMissionNode : public rclcpp::Node
{
public:
  CompetitionMissionNode()
  : Node("competition_mission_manager")
  {
    read_parameters();
    validate_parameters();
    arrival_tracker_ = std::make_unique<ArrivalTracker>(
      arrival_confirm_time_sec_, reach_lost_grace_time_sec_);

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable().transient_local();

    pid_target_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pid/target_pose", reliable_qos);
    current_waypoint_publisher_ = create_publisher<uav_interfaces::msg::Waypoint>(
      "/waypoint/current", retained_qos);
    mission_status_publisher_ = create_publisher<uav_interfaces::msg::MissionStatus>(
      "/mission/status", retained_qos);
    land_request_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/mission/land_request", retained_qos);
    phase_publisher_ = create_publisher<std_msgs::msg::String>(
      "/competition/phase", retained_qos);
    vision_assist_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/competition/vision_assist_requested", retained_qos);
    payload_drop_request_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/payload/drop_request", retained_qos);
    active_task_mode_publisher_ = create_publisher<std_msgs::msg::UInt8>(
      "/competition/active_task_mode", retained_qos);

    task_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "/competition/task_mode", retained_qos,
      std::bind(&CompetitionMissionNode::task_mode_callback, this, std::placeholders::_1));
    start_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/competition/start", reliable_qos,
      std::bind(&CompetitionMissionNode::start_callback, this, std::placeholders::_1));
    drop_window_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/car/drop_window", reliable_qos,
      std::bind(&CompetitionMissionNode::drop_window_callback, this, std::placeholders::_1));
    lap_complete_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/car/lap_complete", reliable_qos,
      std::bind(&CompetitionMissionNode::lap_complete_callback, this, std::placeholders::_1));
    payload_drop_completed_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/payload/drop_completed", reliable_qos,
      std::bind(
        &CompetitionMissionNode::payload_drop_completed_callback,
        this, std::placeholders::_1));
    payload_drop_failed_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/payload/drop_failed", reliable_qos,
      std::bind(
        &CompetitionMissionNode::payload_drop_failed_callback,
        this, std::placeholders::_1));
    vision_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/vision/target_valid", reliable_qos,
      std::bind(
        &CompetitionMissionNode::vision_valid_callback, this, std::placeholders::_1));
    visual_control_active_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/pid/visual_control_active", reliable_qos,
      std::bind(
        &CompetitionMissionNode::visual_control_active_callback,
        this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/map/current_pose", reliable_qos,
      std::bind(&CompetitionMissionNode::pose_callback, this, std::placeholders::_1));
    pose_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/map/current_pose_valid", reliable_qos,
      std::bind(
        &CompetitionMissionNode::pose_valid_callback, this, std::placeholders::_1));
    pid_reached_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/pid/target_reached", reliable_qos,
      std::bind(
        &CompetitionMissionNode::pid_reached_callback, this, std::placeholders::_1));
    flight_status_subscription_ = create_subscription<uav_interfaces::msg::FlightStatus>(
      "/flight/status", reliable_qos,
      std::bind(
        &CompetitionMissionNode::flight_status_callback, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / manager_rate_hz_));
    manager_timer_ = create_wall_timer(
      timer_period, std::bind(&CompetitionMissionNode::manager_timer_callback, this));

    if (auto_start_for_test_) {
      selected_task_mode_ = default_task_mode_;
      task_mode_received_ = true;
      start_requested_ = true;
      set_phase(CompetitionPhase::WAIT_READY);
      RCLCPP_INFO(
        get_logger(),
        "auto_start_for_test=true：使用默认任务%u，仅允许无桨地面联调",
        selected_task_mode_);
    } else {
      set_phase(CompetitionPhase::WAIT_START);
    }
    publish_outputs();

    RCLCPP_INFO(
      get_logger(),
      "D题任务管理器已启动：等待小车task_mode+START，高度=%.2f m，右移=%.2f m，"
      "前追=%.2f m，task1 TAKEOFF成功后悬停=%.2f s（任务推进不使用HR），"
      "SAFE_HOLD锁存=%s",
      target_height_m_, std::abs(lateral_offset_m_), forward_chase_distance_m_,
      task1_takeoff_hover_time_sec_, enable_safe_hold_latch_ ? "启用" : "禁用");
  }

private:
  void read_parameters()
  {
    auto_start_for_test_ = declare_parameter<bool>("auto_start_for_test", false);
    right_shift_land_test_ = declare_parameter<bool>("right_shift_land_test", false);
    default_task_mode_parameter_ = declare_parameter<int64_t>("default_task_mode", 1);
    require_task_mode_from_car_ =
      declare_parameter<bool>("require_task_mode_from_car", true);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    manager_rate_hz_ = declare_parameter<double>("manager_rate_hz", 20.0);
    print_status_ = declare_parameter<bool>("print_status", true);

    target_height_m_ = declare_parameter<double>("target_height_m", 1.23);
    task1_takeoff_hover_time_sec_ =
      declare_parameter<double>("task1_takeoff_hover_time_sec", 2.0);
    takeoff_completion_timeout_sec_ =
      declare_parameter<double>("takeoff_completion_timeout_sec", 20.0);
    task2_takeoff_hover_time_sec_ =
      declare_parameter<double>("task2_takeoff_hover_time_sec", 0.0);
    lateral_offset_m_ = declare_parameter<double>("lateral_offset_m", -0.375);
    forward_chase_distance_m_ =
      declare_parameter<double>("forward_chase_distance_m", 2.75);
    lateral_hold_time_sec_ = declare_parameter<double>("lateral_hold_time_sec", 0.3);
    vision_confirm_time_sec_ = declare_parameter<double>("vision_confirm_time_sec", 0.3);
    drop_alignment_time_sec_ = declare_parameter<double>("drop_alignment_time_sec", 0.2);
    drop_action_timeout_sec_ = declare_parameter<double>("drop_action_timeout_sec", 10.0);
    home_hold_time_sec_ = declare_parameter<double>("home_hold_time_sec", 0.5);
    task2_land_align_time_sec_ = declare_parameter<double>("task2_land_align_time_sec", 0.5);
    task2_min_descent_time_sec_ = declare_parameter<double>("task2_min_descent_time_sec", 1.0);
    task2_on_car_hold_time_sec_ =
      declare_parameter<double>("task2_on_car_hold_time_sec", 5.0);
    task2_landed_stability_grace_sec_ =
      declare_parameter<double>("task2_landed_stability_grace_sec", 0.25);
    vision_timeout_ms_ = declare_parameter<int>("vision_timeout_ms", 500);
    drop_window_timeout_ms_ = declare_parameter<int>("drop_window_timeout_ms", 500);
    visual_control_timeout_ms_ = declare_parameter<int>("visual_control_timeout_ms", 500);
    drop_feedback_timeout_ms_ = declare_parameter<int>("drop_feedback_timeout_ms", 1000);
    arrival_confirm_time_sec_ =
      declare_parameter<double>("arrival_confirm_time_sec", 0.5);
    reach_lost_grace_time_sec_ =
      declare_parameter<double>("reach_lost_grace_time_sec", 0.2);
    new_target_guard_time_sec_ =
      declare_parameter<double>("new_target_guard_time_sec", 0.3);
    pose_timeout_ms_ = declare_parameter<int>("pose_timeout_ms", 500);
    pid_feedback_timeout_ms_ = declare_parameter<int>("pid_feedback_timeout_ms", 500);
    flight_status_timeout_ms_ =
      declare_parameter<int>("flight_status_timeout_ms", 500);
    enable_safe_hold_latch_ =
      declare_parameter<bool>("enable_safe_hold_latch", false);
    flight_safe_hold_timeout_sec_ =
      declare_parameter<double>("flight_safe_hold_timeout_sec", 2.0);
    flight_recovery_confirm_time_sec_ =
      declare_parameter<double>("flight_recovery_confirm_time_sec", 0.25);

    route_id_parameter_ = declare_parameter<int64_t>("route_id", 20260729);
    takeoff_waypoint_id_parameter_ =
      declare_parameter<int64_t>("takeoff_waypoint_id", 1001);
    lateral_waypoint_id_parameter_ =
      declare_parameter<int64_t>("lateral_waypoint_id", 1002);
    chase_waypoint_id_parameter_ =
      declare_parameter<int64_t>("chase_waypoint_id", 1003);
    home_waypoint_id_parameter_ =
      declare_parameter<int64_t>("home_waypoint_id", 1004);

    max_abs_offset_m_ = declare_parameter<double>("max_abs_offset_m", 10.0);
    min_target_height_m_ = declare_parameter<double>("min_target_height_m", 0.5);
    max_target_height_m_ = declare_parameter<double>("max_target_height_m", 1.7);
  }

  static uint32_t checked_u32(int64_t value, const std::string & name)
  {
    if (value < 0 || value > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
      throw std::invalid_argument(name + " 超出 uint32 范围");
    }
    return static_cast<uint32_t>(value);
  }

  void validate_parameters()
  {
    const auto nonnegative = [](double value) {
        return std::isfinite(value) && value >= 0.0;
      };

    if (default_task_mode_parameter_ != kTaskPayloadDrop &&
      default_task_mode_parameter_ != kTaskDynamicLanding)
    {
      throw std::invalid_argument("default_task_mode只能为1或2");
    }
    default_task_mode_ = static_cast<std::uint8_t>(default_task_mode_parameter_);
    if (right_shift_land_test_ && !auto_start_for_test_) {
      throw std::invalid_argument(
              "right_shift_land_test只能与auto_start_for_test=true同时使用");
    }
    if (map_frame_.empty()) {
      throw std::invalid_argument("map_frame不能为空");
    }
    if (!std::isfinite(manager_rate_hz_) || manager_rate_hz_ <= 0.0) {
      throw std::invalid_argument("manager_rate_hz必须大于0");
    }
    if (!std::isfinite(min_target_height_m_) ||
      !std::isfinite(max_target_height_m_) ||
      min_target_height_m_ < 0.0 || min_target_height_m_ > max_target_height_m_ ||
      max_target_height_m_ > 1.7)
    {
      throw std::invalid_argument("目标高度安全范围非法");
    }
    if (!std::isfinite(target_height_m_) ||
      target_height_m_ < min_target_height_m_ ||
      target_height_m_ > max_target_height_m_)
    {
      throw std::invalid_argument("target_height_m超出比赛安全范围");
    }
    if (!nonnegative(task1_takeoff_hover_time_sec_) ||
      !nonnegative(task2_takeoff_hover_time_sec_) ||
      !nonnegative(lateral_hold_time_sec_) ||
      !nonnegative(vision_confirm_time_sec_) ||
      !nonnegative(drop_alignment_time_sec_) ||
      !nonnegative(drop_action_timeout_sec_) ||
      !nonnegative(home_hold_time_sec_) ||
      !nonnegative(task2_land_align_time_sec_) ||
      !nonnegative(task2_min_descent_time_sec_) ||
      !nonnegative(task2_on_car_hold_time_sec_) ||
      !nonnegative(task2_landed_stability_grace_sec_) ||
      !nonnegative(takeoff_completion_timeout_sec_) ||
      !nonnegative(arrival_confirm_time_sec_) ||
      !nonnegative(reach_lost_grace_time_sec_) ||
      !nonnegative(new_target_guard_time_sec_) ||
      !nonnegative(flight_safe_hold_timeout_sec_) ||
      !nonnegative(flight_recovery_confirm_time_sec_))
    {
      throw std::invalid_argument("任务确认和停留时间必须为非负有限数");
    }
    if (!std::isfinite(max_abs_offset_m_) || max_abs_offset_m_ <= 0.0 ||
      !std::isfinite(lateral_offset_m_) ||
      !std::isfinite(forward_chase_distance_m_) ||
      std::abs(lateral_offset_m_) > max_abs_offset_m_ ||
      forward_chase_distance_m_ < 0.0 ||
      forward_chase_distance_m_ > max_abs_offset_m_)
    {
      throw std::invalid_argument("横移或前追距离非法");
    }
    if (lateral_offset_m_ >= 0.0) {
      throw std::invalid_argument("本赛场方案要求右移，lateral_offset_m必须小于0");
    }
    if (vision_timeout_ms_ <= 0 || drop_window_timeout_ms_ <= 0 ||
      visual_control_timeout_ms_ <= 0 || drop_feedback_timeout_ms_ <= 0 ||
      pose_timeout_ms_ <= 0 || pid_feedback_timeout_ms_ <= 0 ||
      flight_status_timeout_ms_ <= 0)
    {
      throw std::invalid_argument("所有反馈超时必须大于0");
    }
    if (flight_safe_hold_timeout_sec_ <= 0.0 ||
      flight_recovery_confirm_time_sec_ <= 0.0 ||
      flight_recovery_confirm_time_sec_ >= flight_safe_hold_timeout_sec_)
    {
      throw std::invalid_argument("飞控恢复确认时间必须大于0且小于SAFE_HOLD超时");
    }

    if (takeoff_completion_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("takeoff_completion_timeout_sec must be greater than zero");
    }
    route_id_ = checked_u32(route_id_parameter_, "route_id");
    takeoff_waypoint_id_ = checked_u32(takeoff_waypoint_id_parameter_, "takeoff_waypoint_id");
    lateral_waypoint_id_ = checked_u32(lateral_waypoint_id_parameter_, "lateral_waypoint_id");
    chase_waypoint_id_ = checked_u32(chase_waypoint_id_parameter_, "chase_waypoint_id");
    home_waypoint_id_ = checked_u32(home_waypoint_id_parameter_, "home_waypoint_id");
  }

  static bool quaternion_to_yaw(
    const geometry_msgs::msg::Quaternion & orientation, double & yaw)
  {
    const double norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(norm) || norm < 1.0e-9) {
      return false;
    }
    const double x = orientation.x / norm;
    const double y = orientation.y / norm;
    const double z = orientation.z / norm;
    const double w = orientation.w / norm;
    yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return std::isfinite(yaw);
  }

  static const char * phase_name(CompetitionPhase phase)
  {
    switch (phase) {
      case CompetitionPhase::WAIT_START: return "WAIT_START";
      case CompetitionPhase::WAIT_READY: return "WAIT_READY";
      case CompetitionPhase::TAKEOFF_HOVER: return "TAKEOFF_HOVER";
      case CompetitionPhase::LATERAL_SHIFT: return "RIGHT_SHIFT";
      case CompetitionPhase::FORWARD_CHASE: return "FORWARD_CHASE";
      case CompetitionPhase::TASK1_FOLLOW: return "TASK1_FOLLOW";
      case CompetitionPhase::TASK1_ALIGN_DROP: return "TASK1_ALIGN_DROP";
      case CompetitionPhase::TASK1_DROP: return "TASK1_DROP";
      case CompetitionPhase::TASK1_RETURN_HOME: return "TASK1_RETURN_HOME";
      case CompetitionPhase::TASK1_LAND_HOME: return "TASK1_LAND_HOME";
      case CompetitionPhase::TASK1_COMPLETE: return "TASK1_COMPLETE";
      case CompetitionPhase::TASK2_VISUAL_APPROACH: return "TASK2_VISUAL_APPROACH";
      case CompetitionPhase::TASK2_ALIGN_LAND: return "TASK2_ALIGN_LAND";
      case CompetitionPhase::TASK2_DESCEND: return "TASK2_DESCEND";
      case CompetitionPhase::TASK2_ON_CAR: return "TASK2_ON_CAR";
      case CompetitionPhase::TASK2_HOLD_5S: return "TASK2_HOLD_5S";
      case CompetitionPhase::TASK2_RETAKEOFF: return "TASK2_RETAKEOFF";
      case CompetitionPhase::TASK2_RETURN_HOME: return "TASK2_RETURN_H";
      case CompetitionPhase::TASK2_LAND_HOME: return "TASK2_LAND_H";
      case CompetitionPhase::TASK2_COMPLETE: return "TASK2_COMPLETE";
      case CompetitionPhase::SAFE_HOLD: return "SAFE_HOLD";
      case CompetitionPhase::ERROR: return "ERROR";
    }
    return "UNKNOWN";
  }

  static bool phase_uses_visual_control(CompetitionPhase phase)
  {
    return phase == CompetitionPhase::TASK1_FOLLOW ||
           phase == CompetitionPhase::TASK1_ALIGN_DROP ||
           phase == CompetitionPhase::TASK1_DROP ||
           phase == CompetitionPhase::TASK2_VISUAL_APPROACH ||
           phase == CompetitionPhase::TASK2_ALIGN_LAND ||
           phase == CompetitionPhase::TASK2_DESCEND;
  }

  static bool phase_requires_pid(CompetitionPhase phase)
  {
    return phase == CompetitionPhase::TAKEOFF_HOVER ||
           phase == CompetitionPhase::LATERAL_SHIFT ||
           phase == CompetitionPhase::FORWARD_CHASE ||
           phase_uses_visual_control(phase) ||
           phase == CompetitionPhase::TASK1_RETURN_HOME;
  }

  static bool phase_requires_pose(CompetitionPhase phase)
  {
    // 起飞点已经在START时锁存。TAKEOFF成功后的固定等待计时不依赖
    // Cartographer持续有效，避免短时跳变把已经完成的起飞重新打回保护态。
    // task2一旦视觉接管，BODY x/y由摄像头闭环，飞控自身保持航向。
    // Cartographer继续用于记录，但不再作为task2视觉伴飞、下降或落车计时的推进门槛。
    return phase != CompetitionPhase::TAKEOFF_HOVER &&
           phase != CompetitionPhase::TASK2_VISUAL_APPROACH &&
           phase != CompetitionPhase::TASK2_ALIGN_LAND &&
           phase != CompetitionPhase::TASK2_DESCEND &&
           phase != CompetitionPhase::TASK2_ON_CAR &&
           phase != CompetitionPhase::TASK2_HOLD_5S &&
           phase != CompetitionPhase::TASK1_LAND_HOME &&
           phase != CompetitionPhase::TASK1_COMPLETE &&
           phase != CompetitionPhase::TASK2_LAND_HOME &&
           phase != CompetitionPhase::TASK2_COMPLETE;
  }

  void set_phase(CompetitionPhase new_phase)
  {
    if (phase_ == new_phase && phase_initialized_) {
      return;
    }
    phase_ = new_phase;
    phase_initialized_ = true;
    RCLCPP_INFO(
      get_logger(), "比赛任务%u阶段切换：%s",
      selected_task_mode_, phase_name(phase_));
  }
  bool flight_control_is_healthy(
    const rclcpp::Time & current_time, bool & flight_recent) const
  {
    flight_recent = is_recent(
      has_flight_status_, flight_status_time_, current_time, flight_status_timeout_ms_);
    return flight_recent && flight_status_.communication_ok &&
           !flight_status_.failsafe && flight_status_.error_code == 0U;
  }

  void enter_safe_hold(const rclcpp::Time & current_time, const char * reason)
  {
    if (!enable_safe_hold_latch_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "SAFE_HOLD锁存已禁用：%s；保持阶段%s，反馈恢复后立即继续",
        reason, phase_name(phase_));
      return;
    }
    if (phase_ == CompetitionPhase::SAFE_HOLD || phase_ == CompetitionPhase::ERROR) {
      return;
    }
    safe_hold_resume_phase_ = phase_;
    safe_hold_started_time_ = current_time;
    flight_recovery_timing_ = false;
    target_sent_ = false;
    previous_pid_connected_ = false;
    arrival_tracker_->reset();
    set_phase(CompetitionPhase::SAFE_HOLD);
    RCLCPP_WARN(
      get_logger(),
      "安全关键状态异常，立即进入零速SAFE_HOLD：%s；恢复后返回%s",
      reason, phase_name(safe_hold_resume_phase_));
  }

  void resume_from_safe_hold(const rclcpp::Time & current_time)
  {
    const CompetitionPhase resume_phase = safe_hold_resume_phase_;
    target_sent_ = false;
    previous_pid_connected_ = false;
    has_pid_feedback_ = false;
    pid_reached_ = false;
    arrival_tracker_->reset();
    // TAKEOFF动作成功和悬停窗口均为本flight session内的锁存状态。
    // 短时飞控异常恢复后继续剩余计时，但SAFE_HOLD期间本身不计入3秒。
    if (resume_phase == CompetitionPhase::TAKEOFF_HOVER &&
      takeoff_hover_window_started_)
    {
      takeoff_hover_window_start_time_ = takeoff_hover_window_start_time_ +
        (current_time - safe_hold_started_time_);
    }
    vision_confirming_ = false;
    drop_aligning_ = false;
    task2_alignment_timing_ = false;
    if (resume_phase == CompetitionPhase::TASK1_DROP && drop_request_latched_) {
      drop_requested_time_ = current_time;
    }
    if (resume_phase == CompetitionPhase::TASK2_ALIGN_LAND ||
      resume_phase == CompetitionPhase::TASK2_DESCEND ||
      resume_phase == CompetitionPhase::TASK2_HOLD_5S)
    {
      task2_phase_start_time_ = current_time;
    }
    flight_recovery_timing_ = false;
    set_phase(resume_phase);
    RCLCPP_INFO(
      get_logger(), "安全关键反馈连续恢复正常，退出SAFE_HOLD并返回%s",
      phase_name(resume_phase));
  }


  bool ready_reset_is_safe(const rclcpp::Time & current_time) const
  {
    if (!mission_active_) {
      return true;
    }
    const bool flight_status_is_fresh = is_recent(
      has_flight_status_, flight_status_time_, current_time,
      flight_status_timeout_ms_);
    return flight_status_is_fresh && flight_status_.communication_ok &&
           !flight_status_.armed && !flight_status_.airborne && flight_status_.landed;
  }

  void reset_to_ready()
  {
    const bool state_changed = mission_active_ || start_requested_ || task_mode_received_ ||
      selected_task_mode_ != 0U || phase_ != CompetitionPhase::WAIT_START;
    if (!state_changed) {
      return;
    }

    selected_task_mode_ = 0U;
    task_mode_received_ = false;
    start_requested_ = false;
    mission_active_ = false;
    waypoints_.clear();
    current_index_ = 0U;
    target_sent_ = false;
    previous_pid_connected_ = false;
    drop_request_latched_ = false;
    drop_aligning_ = false;
    task2_alignment_timing_ = false;
    task2_stability_lost_timing_ = false;
    takeoff_hover_window_started_ = false;
    takeoff_completion_confirmed_ = false;
    takeoff_action_succeeded_latched_ = false;
    takeoff_flight_session_id_ = 0U;
    takeoff_timeout_latched_ = false;
    flight_recovery_timing_ = false;
    safe_hold_resume_phase_ = CompetitionPhase::WAIT_START;
    has_pid_feedback_ = false;
    pid_reached_ = false;
    has_vision_valid_ = false;
    vision_valid_ = false;
    vision_confirming_ = false;
    has_visual_control_active_ = false;
    visual_control_active_ = false;
    has_drop_window_ = false;
    drop_window_ = false;
    car_lap_complete_ = false;
    has_payload_drop_completed_ = false;
    payload_drop_completed_ = false;
    has_payload_drop_failed_ = false;
    payload_drop_failed_ = false;
    arrival_tracker_->reset();
    set_phase(CompetitionPhase::WAIT_START);
    publish_outputs();
    RCLCPP_INFO(get_logger(), "收到合法READY，已清除上一轮任务并回到WAIT_START");
  }

  void task_mode_callback(const std_msgs::msg::UInt8::SharedPtr message)
  {
    if (message->data == 0U) {
      const auto current_time = now();
      if (!ready_reset_is_safe(current_time)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "飞控状态过期、通信中断或飞行器尚未安全着地，忽略小车READY复位请求");
        return;
      }
      reset_to_ready();
      return;
    }
    if (message->data != kTaskPayloadDrop && message->data != kTaskDynamicLanding) {
      RCLCPP_WARN(get_logger(), "拒绝小车任务模式%u：只能为1或2", message->data);
      return;
    }
    if (mission_active_) {
      if (message->data != selected_task_mode_) {
        RCLCPP_WARN(
          get_logger(), "任务已开始，忽略小车中途切换到task%u", message->data);
      }
      return;
    }
    selected_task_mode_ = message->data;
    task_mode_received_ = true;
    RCLCPP_INFO(get_logger(), "收到小车任务选择：task%u", selected_task_mode_);
  }

  void start_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data || mission_active_) {
      return;
    }
    if (!start_requested_) {
      if (task_mode_received_) {
        RCLCPP_INFO(
          get_logger(), "收到小车START，准备执行task%u", selected_task_mode_);
      } else {
        RCLCPP_WARN(get_logger(), "收到START但尚未收到小车task_mode，保持等待");
      }
    }
    start_requested_ = true;
    set_phase(CompetitionPhase::WAIT_READY);
  }

  void drop_window_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    drop_window_ = message->data;
    has_drop_window_ = true;
    drop_window_time_ = now();
  }

  void lap_complete_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    car_lap_complete_ = message->data;
  }

  void payload_drop_completed_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    payload_drop_completed_ = message->data;
    has_payload_drop_completed_ = true;
    payload_drop_completed_time_ = now();
  }

  void payload_drop_failed_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    payload_drop_failed_ = message->data;
    has_payload_drop_failed_ = true;
    payload_drop_failed_time_ = now();
  }

  void vision_valid_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    vision_valid_ = message->data;
    has_vision_valid_ = true;
    vision_valid_time_ = now();
    if (!vision_valid_) {
      vision_confirming_ = false;
    }
  }

  void visual_control_active_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    visual_control_active_ = message->data;
    has_visual_control_active_ = true;
    visual_control_active_time_ = now();
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    double yaw = 0.0;
    if (message->header.frame_id != map_frame_ ||
      !std::isfinite(message->pose.position.x) ||
      !std::isfinite(message->pose.position.y) ||
      !quaternion_to_yaw(message->pose.orientation, yaw))
    {
      has_pose_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拒绝比赛当前位姿：frame、x/y或四元数非法");
      return;
    }
    current_pose_ = *message;
    current_yaw_ = yaw;
    pose_received_time_ = now();
    has_pose_ = true;
  }

  void pose_valid_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    pose_valid_ = message->data;
    has_pose_valid_ = true;
    pose_valid_time_ = now();
  }

  void pid_reached_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    pid_reached_ = message->data;
    has_pid_feedback_ = true;
    pid_feedback_time_ = now();
  }

  void flight_status_callback(const uav_interfaces::msg::FlightStatus::SharedPtr message)
  {
    flight_status_ = *message;
    has_flight_status_ = true;
    flight_status_time_ = now();
  }

  static bool is_recent(
    bool received, const rclcpp::Time & stamp, const rclcpp::Time & current_time,
    int timeout_ms)
  {
    if (!received) {
      return false;
    }
    const double age_ms = (current_time - stamp).seconds() * 1000.0;
    return std::isfinite(age_ms) && age_ms >= 0.0 &&
           age_ms <= static_cast<double>(timeout_ms);
  }

  bool pose_is_ready(const rclcpp::Time & current_time) const
  {
    return is_recent(has_pose_, pose_received_time_, current_time, pose_timeout_ms_) &&
           is_recent(has_pose_valid_, pose_valid_time_, current_time, pose_timeout_ms_) &&
           pose_valid_;
  }

  bool flight_is_safe_and_fresh(const rclcpp::Time & current_time) const
  {
    return is_recent(
      has_flight_status_, flight_status_time_, current_time,
      flight_status_timeout_ms_) &&
           flight_status_.communication_ok && !flight_status_.failsafe &&
           flight_status_.error_code == 0U;
  }

  bool vision_is_fresh(const rclcpp::Time & current_time) const
  {
    return is_recent(
      has_vision_valid_, vision_valid_time_, current_time, vision_timeout_ms_) &&
           vision_valid_;
  }

  bool drop_window_is_fresh(const rclcpp::Time & current_time) const
  {
    return is_recent(
      has_drop_window_, drop_window_time_, current_time, drop_window_timeout_ms_) &&
           drop_window_;
  }

  bool visual_control_is_fresh(const rclcpp::Time & current_time) const
  {
    return is_recent(
      has_visual_control_active_, visual_control_active_time_, current_time,
      visual_control_timeout_ms_) && visual_control_active_;
  }

  uav_interfaces::msg::Waypoint make_waypoint(
    uint32_t id, double forward_m, double left_m, double hold_time) const
  {
    const auto target = body_relative_target(
      start_x_, start_y_, start_yaw_, forward_m, left_m);
    uav_interfaces::msg::Waypoint waypoint;
    waypoint.id = id;
    waypoint.x = target.x;
    waypoint.y = target.y;
    waypoint.yaw = target.yaw;
    waypoint.target_height = target_height_m_;
    waypoint.hold_time = hold_time;
    return waypoint;
  }

  double takeoff_hover_time_for_selected_task() const
  {
    return selected_task_mode_ == kTaskPayloadDrop ?
           task1_takeoff_hover_time_sec_ : task2_takeoff_hover_time_sec_;
  }

  bool takeoff_action_is_complete() const
  {
    // V1没有离散action反馈；V2使用当前flight session的TAKEOFF成功锁存。
    // 树莓派任务推进只消费SUCCEEDED，不再消费实时HR。
    if (flight_status_.protocol_version != uav_interfaces::msg::FlightStatus::PROTOCOL_V2) {
      return true;
    }
    return takeoff_action_succeeded_latched_;
  }

  void update_takeoff_action_success_latch(const rclcpp::Time & current_time)
  {
    if (flight_status_.protocol_version != uav_interfaces::msg::FlightStatus::PROTOCOL_V2 ||
      !flight_is_safe_and_fresh(current_time) ||
      flight_status_.active_session_id == 0U ||
      flight_status_.active_action_id != takeoff_action_id_)
    {
      return;
    }

    if (takeoff_flight_session_id_ == 0U) {
      takeoff_flight_session_id_ = flight_status_.active_session_id;
    }
    if (flight_status_.active_session_id != takeoff_flight_session_id_ ||
      flight_status_.action_result !=
      uav_interfaces::msg::FlightStatus::ACTION_SUCCEEDED)
    {
      return;
    }

    if (!takeoff_action_succeeded_latched_) {
      takeoff_action_succeeded_latched_ = true;
      takeoff_completion_confirmed_ = true;
      RCLCPP_INFO(
        get_logger(),
        "已锁存flight session=0x%08X的TAKEOFF成功；后续任务推进不再读取HR",
        takeoff_flight_session_id_);
    }
  }

  bool takeoff_feedback_is_complete(const rclcpp::Time & current_time) const
  {
    const bool guard_finished = target_sent_ &&
      (current_time - target_sent_time_).seconds() >= new_target_guard_time_sec_;
    return guard_finished && flight_is_safe_and_fresh(current_time) &&
           takeoff_action_is_complete();
  }

  bool takeoff_is_complete(const rclcpp::Time & current_time)
  {
    const bool action_ready = takeoff_feedback_is_complete(current_time);
    if (action_ready) {
      takeoff_completion_confirmed_ = true;
    }

    const bool takeoff_action_ready =
      flight_status_.protocol_version == uav_interfaces::msg::FlightStatus::PROTOCOL_V2 ?
      takeoff_action_succeeded_latched_ : action_ready;

    // task2没有额外3秒悬停要求；TAKEOFF动作成功后立即进入右移。
    if (selected_task_mode_ == kTaskDynamicLanding) {
      return takeoff_action_ready;
    }

    // task1当前只保留一个阶段推进条件：同一flight session的TAKEOFF动作成功。
    // 动作成功后重新完整等待3秒。HR、水平PID到达和实时地图有效性均不参与计时。
    // PID仍可在此阶段输出起飞点定点修正，但它只负责控制，不负责阶段推进。
    if (!takeoff_hover_window_started_) {
      if (!takeoff_action_ready) {
        return false;
      }
      takeoff_hover_window_started_ = true;
      takeoff_hover_window_start_time_ = current_time;
      RCLCPP_INFO(
        get_logger(),
        "task1 TAKEOFF成功已锁存，开始独立%.2f秒等待计时；HR/PID/定位不作为右移门槛",
        task1_takeoff_hover_time_sec_);
    }
    return (current_time - takeoff_hover_window_start_time_).seconds() >=
           task1_takeoff_hover_time_sec_;
  }

  void start_mission()
  {
    start_x_ = current_pose_.pose.position.x;
    start_y_ = current_pose_.pose.position.y;
    start_yaw_ = current_yaw_;

    waypoints_.clear();
    waypoints_.push_back(make_waypoint(
      takeoff_waypoint_id_, 0.0, 0.0, takeoff_hover_time_for_selected_task()));
    waypoints_.push_back(make_waypoint(
      lateral_waypoint_id_, 0.0, lateral_offset_m_, lateral_hold_time_sec_));
    waypoints_.push_back(make_waypoint(
      chase_waypoint_id_, forward_chase_distance_m_, lateral_offset_m_, 0.0));
    waypoints_.push_back(make_waypoint(
      home_waypoint_id_, 0.0, 0.0, home_hold_time_sec_));

    mission_active_ = true;
    mission_start_time_ = now();
    takeoff_phase_start_time_ = mission_start_time_;
    current_index_ = 0U;
    target_sent_ = false;
    previous_pid_connected_ = false;
    drop_request_latched_ = false;
    drop_aligning_ = false;
    has_payload_drop_failed_ = false;
    payload_drop_failed_ = false;
    takeoff_hover_window_started_ = false;
    takeoff_completion_confirmed_ = false;
    takeoff_action_succeeded_latched_ = false;
    takeoff_flight_session_id_ =
      flight_status_.protocol_version == uav_interfaces::msg::FlightStatus::PROTOCOL_V2 ?
      flight_status_.active_session_id : 0U;
    takeoff_timeout_latched_ = false;
    arrival_tracker_->reset();
    set_phase(CompetitionPhase::TAKEOFF_HOVER);
    publish_current_waypoint();

    RCLCPP_INFO(
      get_logger(),
      "锁定task%u起始位姿：(%.3f, %.3f, yaw=%.3f)；右移目标=(%.3f, %.3f)，追逐目标=(%.3f, %.3f)",
      selected_task_mode_, start_x_, start_y_, start_yaw_,
      waypoints_[1].x, waypoints_[1].y, waypoints_[2].x, waypoints_[2].y);
  }

  void publish_current_waypoint()
  {
    if (mission_active_ && current_index_ < waypoints_.size()) {
      current_waypoint_publisher_->publish(waypoints_[current_index_]);
    }
  }

  void send_current_pid_target()
  {
    if (!mission_active_ || current_index_ >= waypoints_.size()) {
      return;
    }
    const auto & waypoint = waypoints_[current_index_];
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = map_frame_;
    target.pose.position.x = waypoint.x;
    target.pose.position.y = waypoint.y;
    target.pose.orientation.z = std::sin(waypoint.yaw * 0.5);
    target.pose.orientation.w = std::cos(waypoint.yaw * 0.5);
    pid_target_publisher_->publish(target);

    target_sent_ = true;
    target_sent_time_ = now();
    has_pid_feedback_ = false;
    pid_reached_ = false;
    arrival_tracker_->reset();
    RCLCPP_INFO(
      get_logger(), "发送比赛目标 id=%u：(%.3f, %.3f, yaw=%.3f)，高度=%.2f",
      waypoint.id, waypoint.x, waypoint.y, waypoint.yaw, waypoint.target_height);
  }

  void advance_to(std::size_t index, CompetitionPhase phase)
  {
    current_index_ = index;
    target_sent_ = false;
    arrival_tracker_->reset();
    set_phase(phase);
    publish_current_waypoint();
    send_current_pid_target();
  }

  bool waypoint_is_complete(const rclcpp::Time & current_time, double hold_time)
  {
    const bool guard_finished =
      (current_time - target_sent_time_).seconds() >= new_target_guard_time_sec_;
    const bool pid_recent = is_recent(
      has_pid_feedback_, pid_feedback_time_, current_time, pid_feedback_timeout_ms_);
    const bool fully_reached = guard_finished && pid_recent && pid_reached_ &&
      flight_is_safe_and_fresh(current_time) && flight_status_.height_valid;
    return arrival_tracker_->update(
      fully_reached, current_time.seconds(), std::max(0.0, hold_time));
  }

  void enter_visual_phase_for_task()
  {
    if (selected_task_mode_ == kTaskPayloadDrop) {
      set_phase(CompetitionPhase::TASK1_FOLLOW);
      RCLCPP_INFO(get_logger(), "task1已形成视觉伴飞，等待小车C-D投放窗口");
    } else {
      set_phase(CompetitionPhase::TASK2_VISUAL_APPROACH);
      RCLCPP_INFO(get_logger(), "task2已进入视觉接近，等待C点减速完成后车顶对准");
    }
  }

  bool drop_alignment_conditions_met(const rclcpp::Time & current_time) const
  {
    const bool pid_recent = is_recent(
      has_pid_feedback_, pid_feedback_time_, current_time, pid_feedback_timeout_ms_);
    return drop_window_is_fresh(current_time) && vision_is_fresh(current_time) &&
           visual_control_is_fresh(current_time) && pid_recent && pid_reached_ &&
           flight_is_safe_and_fresh(current_time) && flight_status_.height_valid;
  }

  bool task2_landing_conditions_met(const rclcpp::Time & current_time) const
  {
    const bool pid_recent = is_recent(
      has_pid_feedback_, pid_feedback_time_, current_time, pid_feedback_timeout_ms_);
    return drop_window_is_fresh(current_time) && vision_is_fresh(current_time) &&
           visual_control_is_fresh(current_time) && pid_recent && pid_reached_ &&
           flight_is_safe_and_fresh(current_time) && flight_status_.height_valid;
  }

  void update_task2(const rclcpp::Time & current_time)
  {
    const bool final_land_active = flight_is_safe_and_fresh(current_time) &&
      flight_status_.active_action_id == 2U &&
      (flight_status_.current_mode == uav_interfaces::msg::FlightStatus::FINAL_LAND ||
      flight_status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_PENDING ||
      flight_status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_SUCCEEDED);
    if ((phase_ == CompetitionPhase::TASK2_VISUAL_APPROACH ||
      phase_ == CompetitionPhase::TASK2_ALIGN_LAND ||
      phase_ == CompetitionPhase::TASK2_DESCEND) && !vision_is_fresh(current_time) &&
      !(phase_ == CompetitionPhase::TASK2_DESCEND && final_land_active))
    {
      if (phase_ == CompetitionPhase::TASK2_DESCEND) {
        // 视觉失效时立即停止盲降，回到安全定高对准。
        task2_alignment_timing_ = false;
        set_phase(CompetitionPhase::TASK2_ALIGN_LAND);
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "task2视觉目标失效：停止下降并保持安全定高，等待恢复");
      return;
    }
    if (phase_ == CompetitionPhase::TASK2_VISUAL_APPROACH && drop_window_is_fresh(current_time)) {
      set_phase(CompetitionPhase::TASK2_ALIGN_LAND);
      task2_alignment_timing_ = false;
      RCLCPP_INFO(get_logger(), "task2收到C点减速完成，开始车顶视觉对准");
      return;
    }
    if (phase_ == CompetitionPhase::TASK2_ALIGN_LAND) {
      if (!task2_landing_conditions_met(current_time)) {
        task2_alignment_timing_ = false;
        return;
      }
      if (!task2_alignment_timing_) {
        task2_alignment_timing_ = true;
        task2_phase_start_time_ = current_time;
        return;
      }
      if ((current_time - task2_phase_start_time_).seconds() >= task2_land_align_time_sec_) {
        set_phase(CompetitionPhase::TASK2_DESCEND);
        task2_phase_start_time_ = current_time;
        RCLCPP_INFO(get_logger(), "task2车顶对准确认，开始V2精密下降");
      }
      return;
    }
    if (phase_ == CompetitionPhase::TASK2_DESCEND) {
      if ((current_time - task2_phase_start_time_).seconds() >= task2_min_descent_time_sec_ &&
        flight_is_safe_and_fresh(current_time) &&
        flight_status_.active_action_id == 2U &&
        flight_status_.action_result ==
        uav_interfaces::msg::FlightStatus::ACTION_SUCCEEDED &&
        flight_status_.landed)
      {
        set_phase(CompetitionPhase::TASK2_ON_CAR);
        task2_phase_start_time_ = current_time;
        task2_stability_lost_timing_ = false;
        RCLCPP_INFO(get_logger(), "task2飞控报告已可靠着地，落车状态已确认");
      }
      return;
    }
    if (phase_ == CompetitionPhase::TASK2_ON_CAR) {
      set_phase(CompetitionPhase::TASK2_HOLD_5S);
      RCLCPP_INFO(get_logger(), "已落车，稳定确认中");
      return;
    }
    if (phase_ == CompetitionPhase::TASK2_HOLD_5S) {
      const bool landed_still_stable = flight_is_safe_and_fresh(current_time) &&
        flight_status_.active_action_id == 2U &&
        flight_status_.action_result ==
        uav_interfaces::msg::FlightStatus::ACTION_SUCCEEDED &&
        flight_status_.landed && !flight_status_.airborne;
      if (!landed_still_stable) {
        if (!task2_stability_lost_timing_) {
          task2_stability_lost_timing_ = true;
          task2_stability_lost_time_ = current_time;
        }
        if ((current_time - task2_stability_lost_time_).seconds() >=
          task2_landed_stability_grace_sec_)
        {
          // 持续不稳定超过宽限后重新累计完整5秒。
          task2_phase_start_time_ = current_time;
        }
        return;
      }
      if (task2_stability_lost_timing_) {
        const double lost_sec = (current_time - task2_stability_lost_time_).seconds();
        if (lost_sec < task2_landed_stability_grace_sec_) {
          // 短暂遥测抖动只暂停计时，不把这段时间计入稳定5秒。
          task2_phase_start_time_ = task2_phase_start_time_ +
            (current_time - task2_stability_lost_time_);
        } else {
          task2_phase_start_time_ = current_time;
        }
        task2_stability_lost_timing_ = false;
      }
      if ((current_time - task2_phase_start_time_).seconds() < task2_on_car_hold_time_sec_) {
        return;
      }
      set_phase(CompetitionPhase::TASK2_COMPLETE);
      RCLCPP_INFO(get_logger(), "已稳定：task2在车顶结束，不再复飞返航");
      return;
    }
  }

  void update_vision_and_task1(const rclcpp::Time & current_time)
  {
    if (phase_ == CompetitionPhase::FORWARD_CHASE) {
      if (!vision_is_fresh(current_time)) {
        vision_confirming_ = false;
        return;
      }
      if (!vision_confirming_) {
        vision_confirming_ = true;
        vision_confirm_start_time_ = current_time;
        return;
      }
      if ((current_time - vision_confirm_start_time_).seconds() >= vision_confirm_time_sec_) {
        enter_visual_phase_for_task();
      }
      return;
    }

    if (phase_uses_visual_control(phase_) && !vision_is_fresh(current_time)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "视觉目标失效或超时：保持视觉模式，PID输出无效零速度等待恢复");
    }

    if (phase_ == CompetitionPhase::TASK1_FOLLOW && drop_window_is_fresh(current_time)) {
      drop_aligning_ = false;
      set_phase(CompetitionPhase::TASK1_ALIGN_DROP);
      RCLCPP_INFO(get_logger(), "小车已进入C-D投放窗口，开始连续对准确认");
      return;
    }

    if (phase_ == CompetitionPhase::TASK1_ALIGN_DROP) {
      if (!drop_window_is_fresh(current_time)) {
        drop_aligning_ = false;
        set_phase(CompetitionPhase::TASK1_FOLLOW);
        RCLCPP_WARN(get_logger(), "投放窗口失效，撤销本次对准等待，不触发投放");
        return;
      }
      if (!drop_alignment_conditions_met(current_time)) {
        drop_aligning_ = false;
        return;
      }
      if (!drop_aligning_) {
        drop_aligning_ = true;
        drop_alignment_start_time_ = current_time;
        return;
      }
      if ((current_time - drop_alignment_start_time_).seconds() >= drop_alignment_time_sec_) {
        drop_request_latched_ = true;
        drop_requested_time_ = current_time;
        set_phase(CompetitionPhase::TASK1_DROP);
        RCLCPP_INFO(get_logger(), "task1投放条件连续满足，发出本轮唯一投放请求");
      }
      return;
    }

    if (phase_ == CompetitionPhase::TASK1_DROP) {
      const bool completed_after_request = has_payload_drop_completed_ &&
        payload_drop_completed_ && payload_drop_completed_time_ >= drop_requested_time_ &&
        is_recent(
          has_payload_drop_completed_, payload_drop_completed_time_, current_time,
          drop_feedback_timeout_ms_);
      if (completed_after_request) {
        drop_request_latched_ = false;
        advance_to(3U, CompetitionPhase::TASK1_RETURN_HOME);
        RCLCPP_INFO(get_logger(), "投放机构确认完成，退出视觉伴飞并返回H点");
        return;
      }
      const bool failed_after_request = has_payload_drop_failed_ &&
        payload_drop_failed_ && payload_drop_failed_time_ >= drop_requested_time_ &&
        is_recent(
          has_payload_drop_failed_, payload_drop_failed_time_, current_time,
          drop_feedback_timeout_ms_);
      if (failed_after_request) {
        drop_request_latched_ = false;
        advance_to(3U, CompetitionPhase::TASK1_RETURN_HOME);
        RCLCPP_ERROR(
          get_logger(),
          "摄像头报告投放失败：已禁止本轮重复投放并立即返回H点");
        return;
      }
      if ((current_time - drop_requested_time_).seconds() > drop_action_timeout_sec_) {
        drop_request_latched_ = false;
        advance_to(3U, CompetitionPhase::TASK1_RETURN_HOME);
        RCLCPP_ERROR(
          get_logger(),
          "投放请求超时未确认：已禁止本轮重复投放并立即返回H点");
        return;
      }
    }
  }

  void manager_timer_callback()
  {
    const rclcpp::Time current_time = now();
    const bool pid_connected = pid_target_publisher_->get_subscription_count() > 0U;

    if (!mission_active_) {
      if (start_requested_) {
        set_phase(CompetitionPhase::WAIT_READY);
        const bool task_ready = task_mode_received_ || !require_task_mode_from_car_;
        if (!task_mode_received_ && !require_task_mode_from_car_) {
          selected_task_mode_ = default_task_mode_;
          task_mode_received_ = true;
        }
        if (task_ready && pose_is_ready(current_time) &&
          flight_is_safe_and_fresh(current_time))
        {
          start_mission();
        }
      } else {
        set_phase(CompetitionPhase::WAIT_START);
      }
      previous_pid_connected_ = pid_connected;
      publish_outputs();
      return;
    }

    if (phase_ == CompetitionPhase::ERROR) {
      publish_outputs();
      return;
    }

    bool flight_recent = false;
    const bool flight_healthy = flight_control_is_healthy(current_time, flight_recent);
    if (phase_ == CompetitionPhase::TASK1_DROP &&
      (!flight_healthy || !pose_is_ready(current_time)))
    {
      drop_request_latched_ = false;
      advance_to(3U, CompetitionPhase::TASK1_RETURN_HOME);
      RCLCPP_ERROR(
        get_logger(),
        "投放阶段飞控或定位状态异常：已终止投放并切换返航；控制恢复后继续返回H点");
      publish_outputs();
      return;
    }
    if (phase_ == CompetitionPhase::SAFE_HOLD) {
      if (takeoff_timeout_latched_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TAKEOFF completion timeout has latched SAFE_HOLD; land safely and send READY to reset");
        publish_outputs();
        return;
      }
      const bool pose_healthy =
        !phase_requires_pose(safe_hold_resume_phase_) || pose_is_ready(current_time);
      if (!flight_healthy || !pose_healthy) {
        flight_recovery_timing_ = false;
        const double held_sec = (current_time - safe_hold_started_time_).seconds();
        if (held_sec >= flight_safe_hold_timeout_sec_) {
          set_phase(CompetitionPhase::ERROR);
          RCLCPP_ERROR(
            get_logger(),
            "SAFE_HOLD持续%.2f秒仍未恢复，进入ERROR，等待安全着地后READY复位",
            held_sec);
          publish_outputs();
          return;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "空中SAFE_HOLD已持续%.2f/%.2f秒：保持零水平速度和安全高度，等待反馈恢复",
          held_sec, flight_safe_hold_timeout_sec_);
        publish_outputs();
        return;
      }
      if (!flight_recovery_timing_) {
        flight_recovery_timing_ = true;
        flight_recovery_start_time_ = current_time;
      }
      if ((current_time - flight_recovery_start_time_).seconds() >=
        flight_recovery_confirm_time_sec_)
      {
        resume_from_safe_hold(current_time);
      }
      publish_outputs();
      return;
    }

    if (!flight_healthy) {
      const char * reason = !flight_recent ? "飞控反馈过期" :
        (!flight_status_.communication_ok ? "飞控通信无效" :
        (flight_status_.failsafe ? "飞控报告failsafe" : "飞控报告错误码"));
      enter_safe_hold(current_time, reason);
      publish_outputs();
      return;
    }

    if (phase_requires_pose(phase_) && !pose_is_ready(current_time)) {
      enter_safe_hold(current_time, "定位失效或超时");
      publish_outputs();
      return;
    }

    if (phase_ == CompetitionPhase::TAKEOFF_HOVER) {
      update_takeoff_action_success_latch(current_time);
      const bool takeoff_completion_seen =
        flight_status_.protocol_version == uav_interfaces::msg::FlightStatus::PROTOCOL_V2 ?
        takeoff_action_succeeded_latched_ : takeoff_feedback_is_complete(current_time);
      if (takeoff_completion_seen) {
        takeoff_completion_confirmed_ = true;
      } else if (!takeoff_completion_confirmed_ &&
        (current_time - takeoff_phase_start_time_).seconds() >=
        takeoff_completion_timeout_sec_)
      {
        takeoff_timeout_latched_ = true;
        enter_safe_hold(current_time, "flight controller TAKEOFF completion feedback timeout");
        RCLCPP_ERROR(
          get_logger(),
          "TAKEOFF_HOVER timed out after %.2f seconds without TAKEOFF action success; SAFE_HOLD is latched and RIGHT_SHIFT is forbidden until a safe READY reset",
          takeoff_completion_timeout_sec_);
        publish_outputs();
        return;
      }
    }

    if (phase_requires_pid(phase_)) {
      if (!pid_connected) {
        target_sent_ = false;
        previous_pid_connected_ = false;
        arrival_tracker_->reset();
        // TAKEOFF_HOVER的PID只用于水平定点修正。节点暂时断开时继续按
        // 已锁存的TAKEOFF成功结果计时，不能因此永远禁止进入右移。
        if (phase_ != CompetitionPhase::TAKEOFF_HOVER) {
          publish_outputs();
          return;
        }
      }
      if (pid_connected && (!previous_pid_connected_ || !target_sent_)) {
        send_current_pid_target();
      }
    }
    previous_pid_connected_ = pid_connected;

    if (phase_ == CompetitionPhase::TAKEOFF_HOVER &&
      takeoff_is_complete(current_time))
    {
      advance_to(1U, CompetitionPhase::LATERAL_SHIFT);
    } else if (phase_ == CompetitionPhase::LATERAL_SHIFT) {
      if (waypoint_is_complete(current_time, lateral_hold_time_sec_)) {
        if (right_shift_land_test_) {
          set_phase(CompetitionPhase::TASK1_LAND_HOME);
          RCLCPP_WARN(get_logger(), "右移降落测试：右移目标已完成，向飞控请求最终降落");
        } else {
          advance_to(2U, CompetitionPhase::FORWARD_CHASE);
        }
      }
    } else if (phase_ == CompetitionPhase::FORWARD_CHASE ||
      phase_uses_visual_control(phase_))
    {
      if (selected_task_mode_ == kTaskDynamicLanding &&
        phase_ != CompetitionPhase::FORWARD_CHASE)
      {
        update_task2(current_time);
      } else {
        update_vision_and_task1(current_time);
      }
    } else if (selected_task_mode_ == kTaskDynamicLanding &&
      (phase_ == CompetitionPhase::TASK2_ON_CAR ||
      phase_ == CompetitionPhase::TASK2_HOLD_5S))
    {
      update_task2(current_time);
    } else if (phase_ == CompetitionPhase::TASK1_RETURN_HOME &&
      waypoint_is_complete(current_time, home_hold_time_sec_))
    {
      set_phase(CompetitionPhase::TASK1_LAND_HOME);
      RCLCPP_INFO(get_logger(), "已到达H点上方，向飞控请求最终降落");
    } else if (phase_ == CompetitionPhase::TASK1_LAND_HOME &&
      flight_recent && flight_status_.land_completed && car_lap_complete_)
    {
      set_phase(CompetitionPhase::TASK1_COMPLETE);
      if (right_shift_land_test_) {
        RCLCPP_INFO(get_logger(), "右移降落测试全部完成");
      } else {
        RCLCPP_INFO(get_logger(), "task1抛投、返航和H点降落全部完成");
      }
    }

    publish_outputs();
    if (print_status_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "D题task=%u 阶段=%s，TAKEOFF锁存=%s，飞控M=%u A=%u/%u H=%.2f，PID到达=%s，视觉=%s，投放窗=%s，通信=%s",
        selected_task_mode_, phase_name(phase_),
        takeoff_action_succeeded_latched_ ? "true" : "false",
        flight_status_.current_mode, flight_status_.active_action_id,
        flight_status_.action_result, flight_status_.current_height,
        pid_reached_ ? "true" : "false",
        vision_is_fresh(current_time) ? "true" : "false",
        drop_window_is_fresh(current_time) ? "true" : "false",
        (flight_recent && flight_status_.communication_ok) ? "true" : "false");
    }
  }

  void publish_outputs()
  {
    const bool route_usable = mission_active_ &&
      phase_ != CompetitionPhase::SAFE_HOLD &&
      phase_ != CompetitionPhase::ERROR &&
      phase_ != CompetitionPhase::TASK1_COMPLETE &&
      phase_ != CompetitionPhase::TASK2_COMPLETE;
    const bool landing_requested = phase_ == CompetitionPhase::TASK1_LAND_HOME ||
      phase_ == CompetitionPhase::TASK2_LAND_HOME;
    const bool navigation_completed = landing_requested ||
      phase_ == CompetitionPhase::TASK1_COMPLETE ||
      phase_ == CompetitionPhase::TASK2_COMPLETE;

    std_msgs::msg::Bool land;
    land.data = landing_requested;
    land_request_publisher_->publish(land);

    std_msgs::msg::Bool vision_assist;
    vision_assist.data = phase_uses_visual_control(phase_);
    vision_assist_publisher_->publish(vision_assist);

    std_msgs::msg::Bool drop_request;
    drop_request.data = drop_request_latched_ &&
      phase_ == CompetitionPhase::TASK1_DROP;
    payload_drop_request_publisher_->publish(drop_request);

    std_msgs::msg::UInt8 task_mode;
    task_mode.data = task_mode_received_ ? selected_task_mode_ : 0U;
    active_task_mode_publisher_->publish(task_mode);

    std_msgs::msg::String phase_message;
    phase_message.data = phase_name(phase_);
    phase_publisher_->publish(phase_message);

    uav_interfaces::msg::MissionStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.route_id = route_id_;
    status.route_valid = route_usable;
    status.target_sent = route_usable && target_sent_;
    status.mission_completed = navigation_completed;

    if (!mission_active_) {
      status.state = uav_interfaces::msg::MissionStatus::WAITING_FOR_ROUTE;
      mission_status_publisher_->publish(status);
      return;
    }

    status.waypoint_count = static_cast<uint32_t>(waypoints_.size());
    status.current_index = static_cast<uint32_t>(current_index_);
    if (current_index_ < waypoints_.size()) {
      status.current_waypoint_id = waypoints_[current_index_].id;
    }
    if (phase_ == CompetitionPhase::ERROR) {
      status.state = uav_interfaces::msg::MissionStatus::ERROR;
    } else if (phase_ == CompetitionPhase::SAFE_HOLD) {
      status.state = uav_interfaces::msg::MissionStatus::WAITING_FOR_PID;
    } else if (navigation_completed) {
      status.state = uav_interfaces::msg::MissionStatus::COMPLETED;
    } else if (!target_sent_) {
      status.state = uav_interfaces::msg::MissionStatus::WAITING_FOR_PID;
    } else if (arrival_tracker_->dwelling()) {
      status.state = uav_interfaces::msg::MissionStatus::DWELLING;
    } else {
      status.state = uav_interfaces::msg::MissionStatus::TRACKING;
    }
    mission_status_publisher_->publish(status);
  }

  bool auto_start_for_test_{false};
  bool right_shift_land_test_{false};
  int64_t default_task_mode_parameter_{1};
  std::uint8_t default_task_mode_{kTaskPayloadDrop};
  bool require_task_mode_from_car_{true};
  std::string map_frame_{"map"};
  double manager_rate_hz_{20.0};
  bool print_status_{true};
  double target_height_m_{1.23};
  double task1_takeoff_hover_time_sec_{2.0};
  double takeoff_completion_timeout_sec_{20.0};
  double task2_takeoff_hover_time_sec_{0.0};
  double lateral_offset_m_{-0.375};
  double forward_chase_distance_m_{2.75};
  double lateral_hold_time_sec_{0.3};
  double vision_confirm_time_sec_{0.3};
  double drop_alignment_time_sec_{0.2};
  double drop_action_timeout_sec_{10.0};
  double home_hold_time_sec_{0.5};
  double task2_land_align_time_sec_{0.5};
  double task2_min_descent_time_sec_{1.0};
  double task2_on_car_hold_time_sec_{5.0};
  int vision_timeout_ms_{500};
  int drop_window_timeout_ms_{500};
  int visual_control_timeout_ms_{500};
  int drop_feedback_timeout_ms_{1000};
  double arrival_confirm_time_sec_{0.5};
  double reach_lost_grace_time_sec_{0.2};
  double new_target_guard_time_sec_{0.3};
  int pose_timeout_ms_{500};
  int pid_feedback_timeout_ms_{500};
  int flight_status_timeout_ms_{500};
  bool enable_safe_hold_latch_{false};
  double flight_safe_hold_timeout_sec_{2.0};
  double task2_landed_stability_grace_sec_{0.25};
  double flight_recovery_confirm_time_sec_{0.25};
  double max_abs_offset_m_{10.0};
  double min_target_height_m_{0.5};
  double max_target_height_m_{1.7};

  std::uint32_t takeoff_action_id_{1U};
  int64_t route_id_parameter_{20260729};
  int64_t takeoff_waypoint_id_parameter_{1001};
  int64_t lateral_waypoint_id_parameter_{1002};
  int64_t chase_waypoint_id_parameter_{1003};
  int64_t home_waypoint_id_parameter_{1004};
  uint32_t route_id_{20260729U};
  uint32_t takeoff_waypoint_id_{1001U};
  uint32_t lateral_waypoint_id_{1002U};
  uint32_t chase_waypoint_id_{1003U};
  uint32_t home_waypoint_id_{1004U};

  CompetitionPhase phase_{CompetitionPhase::WAIT_START};
  CompetitionPhase safe_hold_resume_phase_{CompetitionPhase::WAIT_START};
  bool phase_initialized_{false};
  bool flight_recovery_timing_{false};
  bool task_mode_received_{false};
  std::uint8_t selected_task_mode_{0U};
  bool start_requested_{false};
  bool mission_active_{false};
  std::vector<uav_interfaces::msg::Waypoint> waypoints_;
  std::size_t current_index_{0U};
  bool target_sent_{false};
  bool previous_pid_connected_{false};
  bool drop_request_latched_{false};
  bool drop_aligning_{false};
  bool task2_alignment_timing_{false};
  bool takeoff_hover_window_started_{false};
  bool takeoff_completion_confirmed_{false};
  bool takeoff_action_succeeded_latched_{false};
  std::uint32_t takeoff_flight_session_id_{0U};
  bool takeoff_timeout_latched_{false};
  double start_x_{0.0};
  double start_y_{0.0};
  bool task2_stability_lost_timing_{false};
  rclcpp::Time task2_stability_lost_time_{0, 0, RCL_ROS_TIME};
  double start_yaw_{0.0};
  rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time takeoff_phase_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time takeoff_hover_window_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_sent_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time drop_alignment_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time drop_requested_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time task2_phase_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time safe_hold_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time flight_recovery_start_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<ArrivalTracker> arrival_tracker_;

  bool has_pose_{false};
  bool has_pose_valid_{false};
  bool pose_valid_{false};
  double current_yaw_{0.0};
  geometry_msgs::msg::PoseStamped current_pose_;
  rclcpp::Time pose_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pose_valid_time_{0, 0, RCL_ROS_TIME};
  bool has_pid_feedback_{false};
  bool pid_reached_{false};
  rclcpp::Time pid_feedback_time_{0, 0, RCL_ROS_TIME};
  bool has_flight_status_{false};
  uav_interfaces::msg::FlightStatus flight_status_;
  rclcpp::Time flight_status_time_{0, 0, RCL_ROS_TIME};

  bool has_vision_valid_{false};
  bool vision_valid_{false};
  bool vision_confirming_{false};
  bool has_visual_control_active_{false};
  bool visual_control_active_{false};
  rclcpp::Time vision_valid_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time vision_confirm_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time visual_control_active_time_{0, 0, RCL_ROS_TIME};

  bool has_drop_window_{false};
  bool drop_window_{false};
  bool car_lap_complete_{false};
  rclcpp::Time drop_window_time_{0, 0, RCL_ROS_TIME};
  bool has_payload_drop_completed_{false};
  bool payload_drop_completed_{false};
  rclcpp::Time payload_drop_completed_time_{0, 0, RCL_ROS_TIME};
  bool has_payload_drop_failed_{false};
  bool payload_drop_failed_{false};
  rclcpp::Time payload_drop_failed_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pid_target_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::Waypoint>::SharedPtr current_waypoint_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::MissionStatus>::SharedPtr mission_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr land_request_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_assist_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr payload_drop_request_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_task_mode_publisher_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr task_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drop_window_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lap_complete_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_drop_completed_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_drop_failed_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_valid_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visual_control_active_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pose_valid_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pid_reached_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::FlightStatus>::SharedPtr flight_status_subscription_;
  rclcpp::TimerBase::SharedPtr manager_timer_;
};

}  // namespace waypoint_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypoint_manager::CompetitionMissionNode>());
  rclcpp::shutdown();
  return 0;
}
