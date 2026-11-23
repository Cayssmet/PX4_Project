#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "uav_interfaces/msg/flight_status.hpp"
#include "uav_interfaces/msg/mission_status.hpp"
#include "uav_interfaces/msg/waypoint.hpp"
#include "uav_interfaces/msg/waypoint_list.hpp"
#include "uav_interfaces/srv/set_waypoint_list.hpp"

#include "waypoint_manager/arrival_tracker.hpp"
#include "waypoint_manager/route_validation.hpp"

namespace waypoint_manager
{

using namespace std::chrono_literals;

/**
 * @brief 航线存储、航点发送和任务进度管理节点。
 *
 * 本节点不计算速度，也不直接操作飞控。它负责：
 * - 保存一整条有序航线；
 * - 原子接收地面站覆盖航线；
 * - 只在必要时向 position_pid_controller 发送当前 x/y/yaw 目标；
 * - 同时向后续飞控桥发布包含 target_height 的完整当前航点；
 * - 合并平面/yaw 到达状态与飞控高度状态，安全地推进到下一航点；
 * - 最后一个航点完整完成后发出 land_request=true。
 */
class WaypointManagerNode : public rclcpp::Node
{
public:
  WaypointManagerNode()
  : Node("waypoint_manager")
  {
    declare_and_read_parameters();
    validate_parameters();

    arrival_tracker_ = std::make_unique<ArrivalTracker>(
      arrival_confirm_time_, reach_lost_grace_time_);

    // PID 目标使用可靠但不保留历史的 QoS。管理器会检测 PID 连接和重连并主动补发。
    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pid_target_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pid/target_pose", reliable_qos);

    // 当前航点、任务状态和降落请求均只需保留最新一条。后启动的飞控桥也能立即读到。
    const auto retained_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable().transient_local();
    current_waypoint_publisher_ = create_publisher<uav_interfaces::msg::Waypoint>(
      "/waypoint/current", retained_qos);
    mission_status_publisher_ = create_publisher<uav_interfaces::msg::MissionStatus>(
      "/mission/status", retained_qos);
    land_request_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/mission/land_request", retained_qos);

    pid_reached_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/pid/target_reached", reliable_qos,
      std::bind(&WaypointManagerNode::pid_reached_callback, this, std::placeholders::_1));
    flight_status_subscription_ = create_subscription<uav_interfaces::msg::FlightStatus>(
      "/flight/status", reliable_qos,
      std::bind(&WaypointManagerNode::flight_status_callback, this, std::placeholders::_1));

    // 地面站桥应调用此服务发送“一整条”航线；通过检查后才一次性替换旧航线。
    set_route_service_ = create_service<uav_interfaces::srv::SetWaypointList>(
      "/waypoint/set_list",
      std::bind(
        &WaypointManagerNode::set_route_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / manager_rate_hz_));
    manager_timer_ = create_wall_timer(
      timer_period, std::bind(&WaypointManagerNode::manager_timer_callback, this));

    load_local_route_candidate();

    // 默认比赛模式下，本地航线只能用于检查和备用，绝不会自行激活。
    if (!require_ground_station_route_ && allow_local_route_for_test_ && local_route_valid_) {
      activate_route(local_route_candidate_, "本地 YAML 测试航线");
    } else {
      mission_state_ = uav_interfaces::msg::MissionStatus::WAITING_FOR_ROUTE;
      publish_land_request(false);
    }

    RCLCPP_INFO(
      get_logger(),
      "航点管理器已启动：比赛模式等待地面站=%s，到达确认=%.2f s，最小停留=%.2f s",
      require_ground_station_route_ ? "true" : "false",
      arrival_confirm_time_, minimum_hold_time_);

    if (require_ground_station_route_) {
      RCLCPP_WARN(
        get_logger(),
        "尚未收到地面站有效航线：当前不会向 PID 或飞控桥激活任何本地航点");
    }
  }

private:
  void declare_and_read_parameters()
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    manager_rate_hz_ = declare_parameter<double>("manager_rate_hz", 20.0);
    print_status_ = declare_parameter<bool>("print_status", true);

    require_ground_station_route_ =
      declare_parameter<bool>("require_ground_station_route", true);
    allow_local_route_for_test_ =
      declare_parameter<bool>("allow_local_route_for_test", false);
    land_on_mission_complete_ =
      declare_parameter<bool>("land_on_mission_complete", true);

    arrival_confirm_time_ = declare_parameter<double>("arrival_confirm_time", 0.7);
    minimum_hold_time_ = declare_parameter<double>("minimum_hold_time", 1.0);
    reach_lost_grace_time_ = declare_parameter<double>("reach_lost_grace_time", 0.3);
    new_target_guard_time_ = declare_parameter<double>("new_target_guard_time", 0.3);
    pid_feedback_timeout_ms_ = declare_parameter<int>("pid_feedback_timeout_ms", 500);
    flight_status_timeout_ms_ = declare_parameter<int>("flight_status_timeout_ms", 500);

    const int max_waypoints = declare_parameter<int>("max_waypoints", 100);
    route_limits_.map_frame = map_frame_;
    route_limits_.max_waypoints = max_waypoints > 0 ?
      static_cast<std::size_t>(max_waypoints) : 0U;
    route_limits_.max_abs_x = declare_parameter<double>("max_abs_x", 1000.0);
    route_limits_.max_abs_y = declare_parameter<double>("max_abs_y", 1000.0);
    route_limits_.min_target_height =
      declare_parameter<double>("min_target_height", 0.0);
    route_limits_.max_target_height =
      declare_parameter<double>("max_target_height", 20.0);
    route_limits_.max_hold_time = declare_parameter<double>("max_hold_time", 120.0);

    local_route_id_ = declare_parameter<int64_t>("local_route_id", 1);
    local_waypoint_ids_ = declare_parameter<std::vector<int64_t>>(
      "local_waypoint_ids", std::vector<int64_t>{1, 2, 3});
    local_x_ = declare_parameter<std::vector<double>>(
      "local_x", std::vector<double>{0.0, 1.0, 1.0});
    local_y_ = declare_parameter<std::vector<double>>(
      "local_y", std::vector<double>{0.0, 0.0, 1.0});
    local_yaw_ = declare_parameter<std::vector<double>>(
      "local_yaw", std::vector<double>{0.0, 0.0, 1.57079632679});
    local_target_height_ = declare_parameter<std::vector<double>>(
      "local_target_height", std::vector<double>{1.0, 1.0, 1.0});
    local_hold_time_ = declare_parameter<std::vector<double>>(
      "local_hold_time", std::vector<double>{1.0, 1.0, 1.0});
  }

  void validate_parameters() const
  {
    const auto valid_nonnegative = [](double value) {
        return std::isfinite(value) && value >= 0.0;
      };

    if (map_frame_.empty()) {
      throw std::invalid_argument("map_frame 不能为空");
    }
    if (!std::isfinite(manager_rate_hz_) || manager_rate_hz_ <= 0.0) {
      throw std::invalid_argument("manager_rate_hz 必须大于 0");
    }
    if (!valid_nonnegative(arrival_confirm_time_) ||
      !valid_nonnegative(minimum_hold_time_) ||
      !valid_nonnegative(reach_lost_grace_time_) ||
      !valid_nonnegative(new_target_guard_time_))
    {
      throw std::invalid_argument("确认、停留、宽限和新目标保护时间必须是非负有限数");
    }
    if (pid_feedback_timeout_ms_ <= 0 || flight_status_timeout_ms_ <= 0) {
      throw std::invalid_argument("反馈超时参数必须大于 0");
    }
    if (route_limits_.max_waypoints == 0U) {
      throw std::invalid_argument("max_waypoints 必须大于 0");
    }
    if (!valid_nonnegative(route_limits_.max_abs_x) ||
      !valid_nonnegative(route_limits_.max_abs_y))
    {
      throw std::invalid_argument("x/y 范围上限必须是非负有限数");
    }
    if (!std::isfinite(route_limits_.min_target_height) ||
      !std::isfinite(route_limits_.max_target_height) ||
      route_limits_.min_target_height > route_limits_.max_target_height)
    {
      throw std::invalid_argument("目标高度上下限非法");
    }
    if (!valid_nonnegative(route_limits_.max_hold_time) ||
      minimum_hold_time_ > route_limits_.max_hold_time)
    {
      throw std::invalid_argument("max_hold_time 必须大于等于 minimum_hold_time");
    }
  }

  /** @brief 从 ROS 参数数组建立本地候选航线，但不一定激活它。 */
  void load_local_route_candidate()
  {
    const std::size_t count = local_waypoint_ids_.size();
    if (local_x_.size() != count || local_y_.size() != count ||
      local_yaw_.size() != count || local_target_height_.size() != count ||
      local_hold_time_.size() != count)
    {
      RCLCPP_ERROR(
        get_logger(),
        "本地航线数组长度不一致：id/x/y/yaw/height/hold 必须一一对应；本地航线不可用");
      return;
    }
    if (local_route_id_ < 0 ||
      local_route_id_ > static_cast<int64_t>(std::numeric_limits<uint32_t>::max()))
    {
      RCLCPP_ERROR(get_logger(), "local_route_id 超出 uint32 范围；本地航线不可用");
      return;
    }

    uav_interfaces::msg::WaypointList candidate;
    candidate.header.stamp = now();
    candidate.header.frame_id = map_frame_;
    candidate.route_id = static_cast<uint32_t>(local_route_id_);
    candidate.waypoints.reserve(count);

    for (std::size_t index = 0; index < count; ++index) {
      if (local_waypoint_ids_[index] < 0 ||
        local_waypoint_ids_[index] >
        static_cast<int64_t>(std::numeric_limits<uint32_t>::max()))
      {
        RCLCPP_ERROR(
          get_logger(), "本地第 %zu 个 waypoint id 超出 uint32 范围", index + 1U);
        return;
      }

      uav_interfaces::msg::Waypoint waypoint;
      waypoint.id = static_cast<uint32_t>(local_waypoint_ids_[index]);
      waypoint.x = local_x_[index];
      waypoint.y = local_y_[index];
      waypoint.yaw = local_yaw_[index];
      waypoint.target_height = local_target_height_[index];
      waypoint.hold_time = local_hold_time_[index];
      candidate.waypoints.push_back(waypoint);
    }

    const auto validation = validate_and_normalize_route(candidate, route_limits_);
    if (!validation.valid) {
      RCLCPP_ERROR(
        get_logger(), "本地 YAML 航线不可用：%s", validation.message.c_str());
      return;
    }

    local_route_candidate_ = validation.normalized_route;
    local_route_valid_ = true;
    RCLCPP_INFO(
      get_logger(), "已检查本地候选航线：route_id=%u，共 %zu 个航点（当前未必激活）",
      local_route_candidate_.route_id, local_route_candidate_.waypoints.size());
  }

  void set_route_callback(
    const std::shared_ptr<uav_interfaces::srv::SetWaypointList::Request> request,
    std::shared_ptr<uav_interfaces::srv::SetWaypointList::Response> response)
  {
    const auto validation = validate_and_normalize_route(request->waypoint_list, route_limits_);
    if (!validation.valid) {
      response->accepted = false;
      response->message = "拒绝新航线：" + validation.message + "；原航线保持不变";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    // 只有整条新航线检查完毕后才执行替换，保证“全接收或全拒绝”。
    activate_route(validation.normalized_route, "地面站服务");
    response->accepted = true;
    response->message = "已接收 route_id=" +
      std::to_string(active_route_.route_id) + "，共 " +
      std::to_string(active_route_.waypoints.size()) + " 个航点";
  }

  void activate_route(
    const uav_interfaces::msg::WaypointList & new_route,
    const std::string & source)
  {
    active_route_ = new_route;
    route_active_ = true;
    current_index_ = 0U;
    mission_completed_ = false;
    target_sent_ = false;
    land_request_ = false;
    has_pid_feedback_ = false;
    has_flight_status_ = false;
    arrival_tracker_->reset();

    publish_current_waypoint();
    publish_land_request(false);

    // 如果 PID 已经在线，航线生效后立刻发送第一个目标；否则等待 DDS 连接。
    const bool pid_connected = pid_target_publisher_->get_subscription_count() > 0U;
    if (pid_connected) {
      send_current_pid_target();
      mission_state_ = uav_interfaces::msg::MissionStatus::TRACKING;
    } else {
      mission_state_ = uav_interfaces::msg::MissionStatus::WAITING_FOR_PID;
    }
    previous_pid_connected_ = pid_connected;

    RCLCPP_INFO(
      get_logger(), "%s已激活航线：route_id=%u，共 %zu 个航点",
      source.c_str(), active_route_.route_id, active_route_.waypoints.size());
  }

  void pid_reached_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    pid_target_reached_ = message->data;
    has_pid_feedback_ = true;
    pid_feedback_received_time_ = now();
  }

  void flight_status_callback(const uav_interfaces::msg::FlightStatus::SharedPtr message)
  {
    latest_flight_status_ = *message;
    has_flight_status_ = true;
    flight_status_received_time_ = now();
  }

  bool feedback_is_recent(
    bool has_feedback, const rclcpp::Time & received_time,
    const rclcpp::Time & current_time, int timeout_ms) const
  {
    if (!has_feedback) {
      return false;
    }
    const double age_ms = (current_time - received_time).seconds() * 1000.0;
    return std::isfinite(age_ms) && age_ms >= 0.0 &&
           age_ms <= static_cast<double>(timeout_ms);
  }

  void manager_timer_callback()
  {
    const rclcpp::Time current_time = now();
    const bool pid_connected = pid_target_publisher_->get_subscription_count() > 0U;

    if (!route_active_) {
      mission_state_ = uav_interfaces::msg::MissionStatus::WAITING_FOR_ROUTE;
      target_sent_ = false;
      land_request_ = false;
      previous_pid_connected_ = pid_connected;
      publish_periodic_outputs();
      return;
    }

    // 任务一旦完成便不再依赖 PID 是否在线，也不会在 PID 重连时重发最后一个目标。
    // 此时持续保持 COMPLETED 和降落请求，直到地面站提交一条新的合法航线。
    if (mission_completed_) {
      mission_state_ = uav_interfaces::msg::MissionStatus::COMPLETED;
      land_request_ = land_on_mission_complete_;
      previous_pid_connected_ = pid_connected;
      publish_periodic_outputs();
      return;
    }

    if (!pid_connected) {
      if (previous_pid_connected_ && target_sent_) {
        RCLCPP_WARN(get_logger(), "检测到 PID 订阅断开：暂停任务推进，等待重连");
      }
      target_sent_ = false;
      mission_state_ = uav_interfaces::msg::MissionStatus::WAITING_FOR_PID;
      arrival_tracker_->reset();
      previous_pid_connected_ = false;
      publish_periodic_outputs();
      return;
    }

    // 从无订阅变为有订阅，或航线替换/航点切换后尚未发送时，只补发一次当前目标。
    if (!previous_pid_connected_ || !target_sent_) {
      send_current_pid_target();
    }
    previous_pid_connected_ = true;

    const bool flight_recent = feedback_is_recent(
      has_flight_status_, flight_status_received_time_, current_time,
      flight_status_timeout_ms_);

    // 飞控明确报告 failsafe 时使用 ERROR 状态并暂停推进；反馈恢复正常后可自动继续。
    if (flight_recent && latest_flight_status_.failsafe) {
      mission_state_ = uav_interfaces::msg::MissionStatus::ERROR;
      arrival_tracker_->reset();
      publish_periodic_outputs();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "飞控报告 failsafe=true：任务推进已暂停，不会切换到下一航点");
      return;
    }

    // 新目标刚发出时忽略旧航点残留的 true，防止瞬间跳过新航点。
    const double target_age = (current_time - target_sent_time_).seconds();
    const bool guard_finished = std::isfinite(target_age) &&
      target_age >= new_target_guard_time_;

    const bool pid_recent = feedback_is_recent(
      has_pid_feedback_, pid_feedback_received_time_, current_time,
      pid_feedback_timeout_ms_);
    const bool fully_reached = guard_finished && pid_recent && flight_recent &&
      pid_target_reached_ && latest_flight_status_.communication_ok &&
      latest_flight_status_.height_valid && !latest_flight_status_.failsafe;

    const double effective_hold_time = std::max(
      minimum_hold_time_, active_route_.waypoints[current_index_].hold_time);
    const bool waypoint_complete = arrival_tracker_->update(
      fully_reached, current_time.seconds(), effective_hold_time);

    if (waypoint_complete) {
      complete_or_advance_waypoint();
    } else {
      mission_state_ = arrival_tracker_->dwelling() ?
        uav_interfaces::msg::MissionStatus::DWELLING :
        uav_interfaces::msg::MissionStatus::TRACKING;
    }

    publish_periodic_outputs();

    if (print_status_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "任务 route=%u 航点=%zu/%zu：PID到达=%s，高度有效=%s，通信=%s，状态=%u",
        active_route_.route_id, current_index_ + 1U, active_route_.waypoints.size(),
        (pid_recent && pid_target_reached_) ? "true" : "false",
        (flight_recent && latest_flight_status_.height_valid) ? "true" : "false",
        (flight_recent && latest_flight_status_.communication_ok) ? "true" : "false",
        static_cast<unsigned int>(mission_state_));
    }
  }

  void complete_or_advance_waypoint()
  {
    const auto completed_id = active_route_.waypoints[current_index_].id;
    if (current_index_ + 1U < active_route_.waypoints.size()) {
      ++current_index_;
      target_sent_ = false;
      arrival_tracker_->reset();
      publish_current_waypoint();
      send_current_pid_target();
      mission_state_ = uav_interfaces::msg::MissionStatus::TRACKING;
      RCLCPP_INFO(
        get_logger(), "航点 id=%u 完成，切换到第 %zu 个航点（id=%u）",
        completed_id, current_index_ + 1U,
        active_route_.waypoints[current_index_].id);
      return;
    }

    mission_completed_ = true;
    mission_state_ = uav_interfaces::msg::MissionStatus::COMPLETED;
    land_request_ = land_on_mission_complete_;
    publish_land_request(land_request_);
    RCLCPP_INFO(
      get_logger(), "最后一个航点 id=%u 已完成，整条航线结束，降落请求=%s",
      completed_id, land_request_ ? "true" : "false");
  }

  void publish_current_waypoint()
  {
    if (!route_active_ || current_index_ >= active_route_.waypoints.size()) {
      return;
    }
    current_waypoint_publisher_->publish(active_route_.waypoints[current_index_]);
  }

  void send_current_pid_target()
  {
    if (!route_active_ || current_index_ >= active_route_.waypoints.size()) {
      return;
    }

    const auto & waypoint = active_route_.waypoints[current_index_];
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = map_frame_;
    target.pose.position.x = waypoint.x;
    target.pose.position.y = waypoint.y;

    // 此系统只使用平面 yaw，因此四元数的 x/y 为 0，z/w 由半角公式直接得到。
    target.pose.orientation.z = std::sin(waypoint.yaw * 0.5);
    target.pose.orientation.w = std::cos(waypoint.yaw * 0.5);
    pid_target_publisher_->publish(target);

    target_sent_ = true;
    target_sent_time_ = rclcpp::Time(target.header.stamp, get_clock()->get_clock_type());

    // 强制等待新目标发出后的新反馈，不能沿用上一航点的“已到达”。
    has_pid_feedback_ = false;
    has_flight_status_ = false;
    pid_target_reached_ = false;
    arrival_tracker_->reset();

    RCLCPP_INFO(
      get_logger(),
      "已向 PID 发送航点 id=%u：x=%.3f m, y=%.3f m, yaw=%.3f rad；目标高度=%.3f m",
      waypoint.id, waypoint.x, waypoint.y, waypoint.yaw, waypoint.target_height);
  }

  void publish_land_request(bool request)
  {
    std_msgs::msg::Bool message;
    message.data = request;
    land_request_publisher_->publish(message);
  }

  void publish_periodic_outputs()
  {
    publish_land_request(land_request_);

    uav_interfaces::msg::MissionStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.state = mission_state_;
    status.route_valid = route_active_;
    status.target_sent = target_sent_;
    status.mission_completed = mission_completed_;

    if (route_active_) {
      status.route_id = active_route_.route_id;
      status.waypoint_count = static_cast<uint32_t>(active_route_.waypoints.size());
      status.current_index = static_cast<uint32_t>(current_index_);
      if (current_index_ < active_route_.waypoints.size()) {
        status.current_waypoint_id = active_route_.waypoints[current_index_].id;
      }
    }
    mission_status_publisher_->publish(status);
  }

  // ---------- 可调参数 ----------
  std::string map_frame_{"map"};
  double manager_rate_hz_{20.0};
  bool print_status_{true};
  bool require_ground_station_route_{true};
  bool allow_local_route_for_test_{false};
  bool land_on_mission_complete_{true};
  double arrival_confirm_time_{0.7};
  double minimum_hold_time_{1.0};
  double reach_lost_grace_time_{0.3};
  double new_target_guard_time_{0.3};
  int pid_feedback_timeout_ms_{500};
  int flight_status_timeout_ms_{500};
  RouteLimits route_limits_;

  // ---------- 本地 YAML 候选航线 ----------
  int64_t local_route_id_{1};
  std::vector<int64_t> local_waypoint_ids_;
  std::vector<double> local_x_;
  std::vector<double> local_y_;
  std::vector<double> local_yaw_;
  std::vector<double> local_target_height_;
  std::vector<double> local_hold_time_;
  bool local_route_valid_{false};
  uav_interfaces::msg::WaypointList local_route_candidate_;

  // ---------- 当前任务状态 ----------
  bool route_active_{false};
  uav_interfaces::msg::WaypointList active_route_;
  std::size_t current_index_{0U};
  uint8_t mission_state_{uav_interfaces::msg::MissionStatus::WAITING_FOR_ROUTE};
  bool target_sent_{false};
  bool mission_completed_{false};
  bool land_request_{false};
  bool previous_pid_connected_{false};
  rclcpp::Time target_sent_time_{0, 0, RCL_ROS_TIME};

  // ---------- PID 与飞控反馈 ----------
  bool has_pid_feedback_{false};
  bool pid_target_reached_{false};
  rclcpp::Time pid_feedback_received_time_{0, 0, RCL_ROS_TIME};
  bool has_flight_status_{false};
  uav_interfaces::msg::FlightStatus latest_flight_status_;
  rclcpp::Time flight_status_received_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<ArrivalTracker> arrival_tracker_;

  // ---------- ROS 2 接口 ----------
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pid_target_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::Waypoint>::SharedPtr current_waypoint_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::MissionStatus>::SharedPtr mission_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr land_request_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pid_reached_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::FlightStatus>::SharedPtr
    flight_status_subscription_;
  rclcpp::Service<uav_interfaces::srv::SetWaypointList>::SharedPtr set_route_service_;
  rclcpp::TimerBase::SharedPtr manager_timer_;
};

}  // namespace waypoint_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypoint_manager::WaypointManagerNode>());
  rclcpp::shutdown();
  return 0;
}
