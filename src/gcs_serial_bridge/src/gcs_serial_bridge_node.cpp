#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "uav_interfaces/msg/car_status.hpp"
#include "uav_interfaces/msg/flight_status.hpp"
#include "uav_interfaces/msg/gcs_status.hpp"
#include "uav_interfaces/msg/uav_status.hpp"

#include "gcs_serial_bridge/protocol.hpp"

namespace gcs_serial_bridge
{
namespace
{
using UavStatusMsg = uav_interfaces::msg::UavStatus;

std::uint8_t phase_code(const std::string & phase)
{
  if (phase == "WAIT_START") {return UavStatusMsg::PHASE_READY;}
  if (phase == "WAIT_READY") {return UavStatusMsg::PHASE_MISSION_START;}
  if (phase == "TAKEOFF_HOVER") {return UavStatusMsg::PHASE_TAKEOFF_HOVER;}
  if (phase == "RIGHT_SHIFT") {return UavStatusMsg::PHASE_RIGHT_SHIFT;}
  if (phase == "FORWARD_CHASE") {return UavStatusMsg::PHASE_FORWARD_CHASE;}
  if (phase == "SEARCH_CAR") {return UavStatusMsg::PHASE_SEARCH_CAR;}
  if (phase == "TASK1_FOLLOW") {return UavStatusMsg::PHASE_FOLLOW_CAR;}
  if (phase == "TASK1_ALIGN_DROP") {return UavStatusMsg::PHASE_TASK1_ALIGN_DROP;}
  if (phase == "TASK1_DROP") {return UavStatusMsg::PHASE_TASK1_DROP;}
  if (phase == "TASK1_RETURN_HOME") {return UavStatusMsg::PHASE_TASK1_RETURN_H;}
  if (phase == "TASK1_LAND_HOME") {return UavStatusMsg::PHASE_TASK1_LAND_H;}
  if (phase == "TASK1_COMPLETE") {return UavStatusMsg::PHASE_TASK1_COMPLETE;}
  if (phase == "TASK2_VISUAL_APPROACH" || phase == "TASK2_ALIGN_LAND") {
    return UavStatusMsg::PHASE_TASK2_ALIGN_LAND;
  }
  if (phase == "TASK2_DESCEND") {return UavStatusMsg::PHASE_TASK2_DESCEND;}
  if (phase == "TASK2_ON_CAR") {return UavStatusMsg::PHASE_TASK2_ON_CAR;}
  if (phase == "TASK2_HOLD_5S") {return UavStatusMsg::PHASE_TASK2_HOLD_5S;}
  if (phase == "TASK2_RETAKEOFF") {return UavStatusMsg::PHASE_TASK2_RETAKEOFF;}
  if (phase == "TASK2_RETURN_H") {return UavStatusMsg::PHASE_TASK2_RETURN_H;}
  if (phase == "TASK2_LAND_H") {return UavStatusMsg::PHASE_TASK2_LAND_H;}
  if (phase == "TASK2_COMPLETE") {return UavStatusMsg::PHASE_TASK2_COMPLETE;}
  if (phase == "SAFE_HOLD") {return UavStatusMsg::PHASE_SAFE_HOLD;}
  if (phase == "ERROR") {return UavStatusMsg::PHASE_ERROR;}
  if (phase == "ABORTED") {return UavStatusMsg::PHASE_ABORTED;}
  return UavStatusMsg::PHASE_ERROR;
}

bool valid_session_task(std::uint16_t session_id, std::uint8_t task_mode)
{
  return (session_id == 0U && task_mode == 0U) ||
         (session_id == 1U && task_mode == 1U) ||
         (session_id == 2U && task_mode == 2U);
}

bool phase_uses_vision(std::uint8_t phase)
{
  return phase == UavStatusMsg::PHASE_FOLLOW_CAR ||
         phase == UavStatusMsg::PHASE_TASK1_ALIGN_DROP ||
         phase == UavStatusMsg::PHASE_TASK1_DROP ||
         phase == UavStatusMsg::PHASE_TASK2_ALIGN_LAND ||
         phase == UavStatusMsg::PHASE_TASK2_DESCEND;
}

bool phase_requires_pose(std::uint8_t phase)
{
  return phase != UavStatusMsg::PHASE_TASK1_LAND_H &&
         phase != UavStatusMsg::PHASE_TASK1_COMPLETE &&
         phase != UavStatusMsg::PHASE_TASK2_LAND_H &&
         phase != UavStatusMsg::PHASE_TASK2_COMPLETE;
}

bool phase_is_complete(std::uint8_t phase)
{
  return phase == UavStatusMsg::PHASE_TASK1_COMPLETE ||
         phase == UavStatusMsg::PHASE_TASK2_COMPLETE;
}

bool phase_is_on_car(std::uint8_t phase)
{
  return phase == UavStatusMsg::PHASE_TASK2_ON_CAR ||
         phase == UavStatusMsg::PHASE_TASK2_HOLD_5S ||
         phase == UavStatusMsg::PHASE_TASK2_RETAKEOFF ||
         phase == UavStatusMsg::PHASE_TASK2_COMPLETE;
}

bool phase_is_final_land(std::uint8_t phase)
{
  return phase == UavStatusMsg::PHASE_TASK1_LAND_H ||
         phase == UavStatusMsg::PHASE_TASK2_LAND_H;
}

bool quaternion_yaw(
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

speed_t baud_constant(int baud)
{
  if (baud == 115200) {
    return B115200;
  }
  throw std::invalid_argument("统一设备协议固定使用115200波特率");
}

std::uint16_t metres_to_u16_mm(float metres)
{
  if (!std::isfinite(metres) || metres <= 0.0F) {
    return 0U;
  }
  const double millimetres = static_cast<double>(metres) * 1000.0;
  return static_cast<std::uint16_t>(
    std::clamp(
      std::llround(millimetres), 0LL,
      static_cast<long long>(std::numeric_limits<std::uint16_t>::max())));
}

}  // namespace

class GcsSerialBridgeNode : public rclcpp::Node
{
public:
  GcsSerialBridgeNode()
  : Node("gcs_serial_bridge")
  {
    device_ = declare_parameter<std::string>("serial_device", "/dev/gcs_bluetooth");
    baud_ = declare_parameter<int>("baud_rate", 115200);
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 10.0);
    heartbeat_timeout_ms_ = declare_parameter<int>("heartbeat_timeout_ms", 1000);
    input_timeout_ms_ = declare_parameter<int>("input_timeout_ms", 500);
    vision_timeout_ms_ = declare_parameter<int>("vision_timeout_ms", 200);
    reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 1000);
    print_status_ = declare_parameter<bool>("print_status", true);
    validate_parameters();
    (void)baud_constant(baud_);

    const auto reliable = rclcpp::QoS(10).reliable();
    const auto retained = rclcpp::QoS(1).reliable().transient_local();

    uav_status_publisher_ = create_publisher<uav_interfaces::msg::UavStatus>(
      "/gcs/uav_status", retained);
    gcs_status_publisher_ = create_publisher<uav_interfaces::msg::GcsStatus>(
      "/gcs/status", retained);
    gcs_link_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/gcs/link_ok", retained);

    phase_subscription_ = create_subscription<std_msgs::msg::String>(
      "/competition/phase", retained,
      [this](const std_msgs::msg::String::SharedPtr message) {
        phase_name_ = message->data;
        phase_received_ = true;
        const std::uint8_t code = phase_code(phase_name_);
        if (code == UavStatusMsg::PHASE_MISSION_START && !mission_started_) {
          start_mission(now());
        }
        if (phase_is_on_car(code)) {
          landed_on_car_latched_ = true;
        }
      });
    task_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "/competition/active_task_mode", retained,
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        if (!mission_started_ && message->data <= 2U) {
          active_task_ = message->data;
        }
      });
    car_status_subscription_ =
      create_subscription<uav_interfaces::msg::CarStatus>(
      "/car/status", retained,
      [this](const uav_interfaces::msg::CarStatus::SharedPtr message) {
        car_status_ = *message;
        car_status_time_ = now();
        has_car_status_ = true;
        if (!message->communication_ok) {
          return;
        }

        if (message->task_mode == 0U && message->session_id == 0U &&
          phase_code(phase_name_) == UavStatusMsg::PHASE_READY)
        {
          reset_to_ready();
          return;
        }

        if (valid_session_task(message->session_id, message->task_mode) &&
          message->task_mode != 0U && !mission_started_)
        {
          active_session_ = message->session_id;
          active_task_ = message->task_mode;
        }

        if ((message->flags & uav_interfaces::msg::CarStatus::FLAG_START_LATCHED) != 0U &&
          valid_session_task(message->session_id, message->task_mode))
        {
          active_session_ = message->session_id;
          active_task_ = message->task_mode;
          if (!mission_started_) {
            start_mission(now());
          }
        }
      });
    car_link_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/car/link_ok", retained,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        car_link_ok_ = message->data;
        car_link_time_ = now();
        has_car_link_ = true;
      });
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/map/current_pose", reliable,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        double yaw = 0.0;
        if (!quaternion_yaw(message->pose.orientation, yaw) ||
          !std::isfinite(message->pose.position.x) ||
          !std::isfinite(message->pose.position.y))
        {
          return;
        }
        pose_ = *message;
        pose_yaw_ = yaw;
        pose_time_ = now();
        has_pose_ = true;
        if (!home_frozen_ && pose_valid_) {
          home_x_ = message->pose.position.x;
          home_y_ = message->pose.position.y;
          home_yaw_ = yaw;
          has_home_ = true;
        }
      });
    pose_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/map/current_pose_valid", reliable,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        pose_valid_ = message->data;
        pose_valid_time_ = now();
        has_pose_valid_ = true;
      });
    flight_subscription_ =
      create_subscription<uav_interfaces::msg::FlightStatus>(
      "/flight/status", reliable,
      [this](const uav_interfaces::msg::FlightStatus::SharedPtr message) {
        flight_status_ = *message;
        flight_time_ = now();
        has_flight_ = true;
      });
    camera_link_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/vision/link_ok", retained,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        camera_link_ok_ = message->data;
        camera_link_time_ = now();
        has_camera_link_ = true;
      });
    visual_control_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/pid/visual_control_active", reliable,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        visual_control_active_ = message->data;
        visual_control_time_ = now();
        has_visual_control_ = true;
      });
    payload_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/payload/drop_completed", reliable,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (message->data) {
          payload_released_latched_ = true;
        }
      });

    io_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this]() {io_tick();});
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / send_rate_hz_)),
      [this]() {send_tick();});
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {heartbeat_tick();});

    publish_link(false);
    publish_gcs_status(false);
    RCLCPP_INFO(
      get_logger(),
      "地面站串口桥v1.0.1启动：%s，115200 8N1，UAV_STATUS %.1fHz；地面站离线不影响飞行",
      device_.c_str(), send_rate_hz_);
  }

  ~GcsSerialBridgeNode() override
  {
    close_port();
  }

private:
  void validate_parameters() const
  {
    if (device_.empty() || !std::isfinite(send_rate_hz_) ||
      send_rate_hz_ <= 0.0 || send_rate_hz_ > 20.0 ||
      heartbeat_timeout_ms_ <= 0 || input_timeout_ms_ <= 0 ||
      vision_timeout_ms_ <= 0 || reconnect_interval_ms_ <= 0)
    {
      throw std::invalid_argument("地面站串口桥参数非法");
    }
  }

  bool fresh(
    bool has_value, const rclcpp::Time & stamp, const rclcpp::Time & current,
    int timeout_ms) const
  {
    if (!has_value) {
      return false;
    }
    const double age_ms = (current - stamp).seconds() * 1000.0;
    return std::isfinite(age_ms) && age_ms >= 0.0 &&
           age_ms <= static_cast<double>(timeout_ms);
  }

  void reset_to_ready()
  {
    active_session_ = 0U;
    active_task_ = 0U;
    mission_started_ = false;
    mission_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    home_frozen_ = false;
    payload_released_latched_ = false;
    landed_on_car_latched_ = false;
  }

  void start_mission(const rclcpp::Time & current)
  {
    mission_started_ = true;
    mission_start_time_ = current;
    if (has_home_) {
      home_frozen_ = true;
    }
  }

  std::uint8_t current_phase(const rclcpp::Time & current) const
  {
    const bool car_recent =
      fresh(has_car_status_, car_status_time_, current, input_timeout_ms_) &&
      car_status_.communication_ok;
    if ((!phase_received_ || phase_name_ == "WAIT_START") && car_recent &&
      car_status_.mission_phase == UavStatusMsg::PHASE_COUNTDOWN_3S)
    {
      return UavStatusMsg::PHASE_COUNTDOWN_3S;
    }
    return phase_received_ ? phase_code(phase_name_) : UavStatusMsg::PHASE_READY;
  }

  std::uint8_t current_task(const rclcpp::Time & current) const
  {
    if (active_task_ != 0U) {
      return active_task_;
    }
    if (fresh(has_car_status_, car_status_time_, current, input_timeout_ms_) &&
      car_status_.communication_ok && car_status_.task_mode <= 2U)
    {
      return car_status_.task_mode;
    }
    return 0U;
  }

  std::uint16_t current_session(const rclcpp::Time & current) const
  {
    if (active_session_ != 0U) {
      return active_session_;
    }
    if (fresh(has_car_status_, car_status_time_, current, input_timeout_ms_) &&
      car_status_.communication_ok && car_status_.session_id <= 2U)
    {
      return car_status_.session_id;
    }
    return 0U;
  }

  std::uint8_t uav_state_for(
    std::uint8_t phase, bool flight_link_ok) const
  {
    if ((flight_link_ok && flight_status_.failsafe) ||
      phase == UavStatusMsg::PHASE_SAFE_HOLD)
    {
      return UavStatusMsg::UAV_FAILSAFE;
    }
    if (phase == UavStatusMsg::PHASE_ERROR ||
      phase == UavStatusMsg::PHASE_ABORTED)
    {
      return UavStatusMsg::UAV_ERROR;
    }
    if (phase == UavStatusMsg::PHASE_READY ||
      phase == UavStatusMsg::PHASE_COUNTDOWN_3S)
    {
      return UavStatusMsg::UAV_STANDBY;
    }
    if (phase == UavStatusMsg::PHASE_MISSION_START ||
      phase == UavStatusMsg::PHASE_TAKEOFF_HOVER)
    {
      return flight_link_ok && flight_status_.active_action_id == 1U &&
             flight_status_.action_result == uav_interfaces::msg::FlightStatus::ACTION_SUCCEEDED ?
             UavStatusMsg::UAV_HOVER : UavStatusMsg::UAV_TAKEOFF;
    }
    if (phase == UavStatusMsg::PHASE_RIGHT_SHIFT ||
      phase == UavStatusMsg::PHASE_FORWARD_CHASE ||
      phase == UavStatusMsg::PHASE_SEARCH_CAR)
    {
      return UavStatusMsg::UAV_MAP_NAVIGATION;
    }
    if (phase == UavStatusMsg::PHASE_FOLLOW_CAR ||
      phase == UavStatusMsg::PHASE_TASK1_ALIGN_DROP ||
      phase == UavStatusMsg::PHASE_TASK1_DROP)
    {
      return UavStatusMsg::UAV_VISUAL_FOLLOW;
    }
    if (phase == UavStatusMsg::PHASE_TASK2_ALIGN_LAND ||
      phase == UavStatusMsg::PHASE_TASK2_DESCEND)
    {
      return UavStatusMsg::UAV_VISUAL_DESCENT;
    }
    if (phase_is_on_car(phase)) {
      return phase == UavStatusMsg::PHASE_TASK2_RETAKEOFF ?
             UavStatusMsg::UAV_RETAKEOFF :
             UavStatusMsg::UAV_LANDED_ON_CAR;
    }
    if (phase == UavStatusMsg::PHASE_TASK1_RETURN_H ||
      phase == UavStatusMsg::PHASE_TASK1_LAND_H ||
      phase == UavStatusMsg::PHASE_TASK2_RETURN_H ||
      phase == UavStatusMsg::PHASE_TASK2_LAND_H)
    {
      return flight_link_ok && flight_status_.landed ?
             UavStatusMsg::UAV_LANDED_H : UavStatusMsg::UAV_RETURNING_H;
    }
    if (phase == UavStatusMsg::PHASE_TASK1_COMPLETE) {
      return UavStatusMsg::UAV_LANDED_H;
    }
    return UavStatusMsg::UAV_ERROR;
  }

  UavStatus build_status(const rclcpp::Time & current)
  {
    UavStatus status;
    status.task_mode = current_task(current);
    status.mission_phase = current_phase(current);

    const bool flight_link_ok =
      fresh(has_flight_, flight_time_, current, input_timeout_ms_) &&
      flight_status_.communication_ok;
    const bool car_link_ok =
      fresh(has_car_link_, car_link_time_, current, input_timeout_ms_) &&
      car_link_ok_ &&
      fresh(has_car_status_, car_status_time_, current, input_timeout_ms_) &&
      car_status_.communication_ok;
    const bool camera_link_ok =
      fresh(has_camera_link_, camera_link_time_, current, vision_timeout_ms_) &&
      camera_link_ok_;
    const bool pose_ok =
      fresh(has_pose_, pose_time_, current, input_timeout_ms_) &&
      fresh(has_pose_valid_, pose_valid_time_, current, input_timeout_ms_) &&
      pose_valid_ && has_home_;
    const bool visual_control_ok =
      fresh(
        has_visual_control_, visual_control_time_, current,
        input_timeout_ms_) && visual_control_active_;

    status.uav_state = uav_state_for(status.mission_phase, flight_link_ok);

    if (pose_ok) {
      RelativePosition relative;
      if (relative_position_mm(
          home_x_, home_y_, home_yaw_,
          pose_.pose.position.x, pose_.pose.position.y, relative))
      {
        status.map_x_mm = relative.x_mm;
        status.map_y_mm = relative.y_mm;
        status.flags |= UavStatusMsg::FLAG_POSE_VALID;
      }
    }
    if (flight_link_ok) {
      status.flags |= UavStatusMsg::FLAG_FLIGHT_LINK_OK;
      if (flight_status_.armed) {
        status.flags |= UavStatusMsg::FLAG_ARMED;
      }
      if (flight_status_.airborne) {
        status.flags |= UavStatusMsg::FLAG_AIRBORNE;
      }
      if (flight_status_.height_valid) {
        status.flags |= UavStatusMsg::FLAG_HEIGHT_VALID;
        status.current_height_mm =
          metres_to_u16_mm(flight_status_.current_height);
      }
      if (flight_status_.failsafe) {
        status.flags |= UavStatusMsg::FLAG_FAILSAFE;
      }
    }
    if (camera_link_ok) {
      status.flags |= UavStatusMsg::FLAG_CAMERA_LINK_OK;
    }
    if (car_link_ok) {
      status.flags |= UavStatusMsg::FLAG_CAR_LINK_OK;
    }
    if (visual_control_ok) {
      status.flags |= UavStatusMsg::FLAG_VISION_CONTROL_ACTIVE;
    }
    if (payload_released_latched_) {
      status.flags |= UavStatusMsg::FLAG_PAYLOAD_RELEASED;
    }
    if (landed_on_car_latched_) {
      status.flags |= UavStatusMsg::FLAG_LANDED_ON_CAR;
    }

    const bool landed_at_h =
      status.mission_phase == UavStatusMsg::PHASE_TASK1_COMPLETE ||
      (flight_link_ok && flight_status_.landed &&
      (status.mission_phase == UavStatusMsg::PHASE_READY ||
      phase_is_final_land(status.mission_phase)));
    if (landed_at_h) {
      status.flags |= UavStatusMsg::FLAG_LANDED_AT_H;
    }
    if (status.mission_phase == UavStatusMsg::PHASE_SAFE_HOLD) {
      status.flags |= UavStatusMsg::FLAG_FAILSAFE;
    }
    if (phase_is_complete(status.mission_phase)) {
      status.flags |= UavStatusMsg::FLAG_TASK_COMPLETE;
    }

    const std::uint16_t session = current_session(current);
    if (!flight_link_ok || flight_status_.failsafe ||
      flight_status_.error_code != 0U)
    {
      status.error_code = UavStatusMsg::FLIGHT_LINK_TIMEOUT;
    } else if (phase_requires_pose(status.mission_phase) &&
      (status.flags & UavStatusMsg::FLAG_POSE_VALID) == 0U)
    {
      status.error_code = UavStatusMsg::POSE_INVALID;
    } else if (!car_link_ok) {
      status.error_code = UavStatusMsg::CAR_TIMEOUT;
    } else if (phase_uses_vision(status.mission_phase) && !camera_link_ok) {
      status.error_code = UavStatusMsg::VISION_TIMEOUT;
    } else if (!valid_session_task(session, status.task_mode)) {
      status.error_code = UavStatusMsg::SESSION_TASK_MISMATCH;
    } else if (status.mission_phase == UavStatusMsg::PHASE_ERROR ||
      status.mission_phase == UavStatusMsg::PHASE_ABORTED)
    {
      status.error_code = UavStatusMsg::SYSTEM_INTERNAL_ERROR;
    } else {
      status.error_code = UavStatusMsg::SYSTEM_OK;
    }

    if (mission_started_) {
      const double elapsed = (current - mission_start_time_).seconds();
      if (std::isfinite(elapsed) && elapsed > 0.0) {
        status.mission_elapsed_s = static_cast<std::uint16_t>(
          std::clamp(
            std::floor(elapsed), 0.0,
            static_cast<double>(std::numeric_limits<std::uint16_t>::max())));
      }
    }
    return status;
  }

  void send_tick()
  {
    const auto current = now();
    const std::uint8_t sequence = tx_sequence_++;
    const std::uint16_t session = current_session(current);
    const UavStatus status = build_status(current);
    publish_uav_status(status, sequence, session);

    if (fd_ >= 0 && tx_buffer_.empty()) {
      tx_buffer_ = encode_frame(
        make_uav_status_frame(status, sequence, session));
      tx_offset_ = 0U;
      flush_tx();
    }

    if (print_status_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "GCS TX=%llu RX=%llu S=%u task=%u phase=0x%02X uav=0x%02X "
        "xy=(%d,%d)mm h=%umm flags=0x%04X err=0x%02X link=%s",
        static_cast<unsigned long long>(tx_count_),
        static_cast<unsigned long long>(rx_count_), session, status.task_mode,
        status.mission_phase, status.uav_state, status.map_x_mm,
        status.map_y_mm, status.current_height_mm, status.flags,
        status.error_code, gcs_link_ok_ ? "true" : "false");
    }
  }

  void publish_uav_status(
    const UavStatus & status, std::uint8_t sequence, std::uint16_t session)
  {
    uav_interfaces::msg::UavStatus message;
    message.header.stamp = now();
    message.header.frame_id = "h_start";
    message.sequence = sequence;
    message.session_id = session;
    message.task_mode = status.task_mode;
    message.mission_phase = status.mission_phase;
    message.uav_state = status.uav_state;
    message.error_code = status.error_code;
    message.flags = status.flags;
    message.map_x_mm = status.map_x_mm;
    message.map_y_mm = status.map_y_mm;
    message.current_height_mm = status.current_height_mm;
    message.mission_elapsed_s = status.mission_elapsed_s;
    uav_status_publisher_->publish(message);
  }

  void publish_gcs_status(bool communication_ok)
  {
    uav_interfaces::msg::GcsStatus message;
    message.header.stamp = now();
    message.header.frame_id = "gcs";
    message.communication_ok = communication_ok;
    message.sequence = latest_gcs_sequence_;
    message.session_id = latest_gcs_session_;
    message.gcs_state = latest_gcs_status_.gcs_state;
    message.mission_phase_echo = latest_gcs_status_.mission_phase_echo;
    message.flags = latest_gcs_status_.flags;
    gcs_status_publisher_->publish(message);
  }

  void publish_link(bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    gcs_link_publisher_->publish(message);
  }

  void heartbeat_tick()
  {
    const bool link_ok =
      fd_ >= 0 &&
      fresh(has_gcs_status_, gcs_status_time_, now(), heartbeat_timeout_ms_);
    if (link_ok != gcs_link_ok_) {
      gcs_link_ok_ = link_ok;
      publish_link(link_ok);
      publish_gcs_status(link_ok);
      if (!link_ok && has_gcs_status_) {
        RCLCPP_WARN(
          get_logger(),
          "超过%dms未收到GCS_STATUS，仅标记地面站离线；飞行任务不受影响",
          heartbeat_timeout_ms_);
      }
    }
  }

  void io_tick()
  {
    const auto current = now();
    if (fd_ < 0) {
      if (!has_reconnect_attempt_ ||
        (current - last_reconnect_attempt_).seconds() * 1000.0 >=
        static_cast<double>(reconnect_interval_ms_))
      {
        last_reconnect_attempt_ = current;
        has_reconnect_attempt_ = true;
        open_port();
      }
      return;
    }

    read_rx();
    flush_tx();
  }

  void open_port()
  {
    const int new_fd = ::open(
      device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (new_fd < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "等待地面站串口%s：%s", device_.c_str(), std::strerror(errno));
      return;
    }

    termios options{};
    if (tcgetattr(new_fd, &options) != 0) {
      const int saved_errno = errno;
      ::close(new_fd);
      RCLCPP_WARN(
        get_logger(), "读取地面站串口参数失败：%s",
        std::strerror(saved_errno));
      return;
    }

    cfmakeraw(&options);
    const speed_t speed = baud_constant(baud_);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(new_fd, TCSANOW, &options) != 0) {
      const int saved_errno = errno;
      ::close(new_fd);
      RCLCPP_WARN(
        get_logger(), "配置地面站串口失败：%s",
        std::strerror(saved_errno));
      return;
    }

    tcflush(new_fd, TCIOFLUSH);
    fd_ = new_fd;
    parser_.reset();
    tx_buffer_.clear();
    tx_offset_ = 0U;
    RCLCPP_INFO(get_logger(), "地面站串口已连接：%s", device_.c_str());
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    parser_.reset();
    tx_buffer_.clear();
    tx_offset_ = 0U;
  }

  void read_rx()
  {
    std::array<std::uint8_t, 512> bytes{};
    while (fd_ >= 0) {
      const ssize_t count = ::read(fd_, bytes.data(), bytes.size());
      if (count > 0) {
        const auto frames =
          parser_.feed(bytes.data(), static_cast<std::size_t>(count));
        for (const auto & frame : frames) {
          handle_frame(frame);
        }
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_WARN(
        get_logger(), "读取地面站串口失败，等待重连：%s",
        std::strerror(errno));
      close_port();
      publish_link(false);
      gcs_link_ok_ = false;
      return;
    }
  }

  void handle_frame(const Frame & frame)
  {
    GcsStatus decoded;
    if (!decode_gcs_status(frame, decoded) ||
      !valid_gcs_status_fields(decoded) ||
      !valid_gcs_session(frame.session_id, current_session(now())))
    {
      return;
    }

    latest_gcs_status_ = decoded;
    latest_gcs_sequence_ = frame.sequence;
    latest_gcs_session_ = frame.session_id;
    gcs_status_time_ = now();
    has_gcs_status_ = true;
    ++rx_count_;
    if (!gcs_link_ok_) {
      gcs_link_ok_ = true;
      publish_link(true);
    }
    publish_gcs_status(true);
  }

  void flush_tx()
  {
    while (fd_ >= 0 && tx_offset_ < tx_buffer_.size()) {
      const ssize_t count = ::write(
        fd_, tx_buffer_.data() + tx_offset_,
        tx_buffer_.size() - tx_offset_);
      if (count > 0) {
        tx_offset_ += static_cast<std::size_t>(count);
        continue;
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      }
      RCLCPP_WARN(
        get_logger(), "写入地面站串口失败，等待重连：%s",
        count < 0 ? std::strerror(errno) : "写入返回0");
      close_port();
      publish_link(false);
      gcs_link_ok_ = false;
      return;
    }

    if (!tx_buffer_.empty() && tx_offset_ == tx_buffer_.size()) {
      ++tx_count_;
      tx_buffer_.clear();
      tx_offset_ = 0U;
    }
  }

  std::string device_;
  int baud_{115200};
  double send_rate_hz_{10.0};
  int heartbeat_timeout_ms_{1000};
  int input_timeout_ms_{500};
  int vision_timeout_ms_{200};
  int reconnect_interval_ms_{1000};
  bool print_status_{true};

  int fd_{-1};
  StreamParser parser_;
  std::vector<std::uint8_t> tx_buffer_;
  std::size_t tx_offset_{0U};
  std::uint8_t tx_sequence_{0U};
  std::uint64_t tx_count_{0U};
  std::uint64_t rx_count_{0U};
  bool has_reconnect_attempt_{false};
  rclcpp::Time last_reconnect_attempt_{0, 0, RCL_ROS_TIME};

  std::string phase_name_{"WAIT_START"};
  bool phase_received_{false};
  std::uint8_t active_task_{0U};
  std::uint16_t active_session_{0U};
  bool mission_started_{false};
  rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};

  uav_interfaces::msg::CarStatus car_status_;
  bool has_car_status_{false};
  rclcpp::Time car_status_time_{0, 0, RCL_ROS_TIME};
  bool has_car_link_{false};
  bool car_link_ok_{false};
  rclcpp::Time car_link_time_{0, 0, RCL_ROS_TIME};

  geometry_msgs::msg::PoseStamped pose_;
  double pose_yaw_{0.0};
  bool has_pose_{false};
  rclcpp::Time pose_time_{0, 0, RCL_ROS_TIME};
  bool has_pose_valid_{false};
  bool pose_valid_{false};
  rclcpp::Time pose_valid_time_{0, 0, RCL_ROS_TIME};
  bool has_home_{false};
  bool home_frozen_{false};
  double home_x_{0.0};
  double home_y_{0.0};
  double home_yaw_{0.0};

  uav_interfaces::msg::FlightStatus flight_status_;
  bool has_flight_{false};
  rclcpp::Time flight_time_{0, 0, RCL_ROS_TIME};

  bool has_camera_link_{false};
  bool camera_link_ok_{false};
  rclcpp::Time camera_link_time_{0, 0, RCL_ROS_TIME};
  bool has_visual_control_{false};
  bool visual_control_active_{false};
  rclcpp::Time visual_control_time_{0, 0, RCL_ROS_TIME};
  bool payload_released_latched_{false};
  bool landed_on_car_latched_{false};

  GcsStatus latest_gcs_status_;
  std::uint8_t latest_gcs_sequence_{0U};
  std::uint16_t latest_gcs_session_{0U};
  bool has_gcs_status_{false};
  rclcpp::Time gcs_status_time_{0, 0, RCL_ROS_TIME};
  bool gcs_link_ok_{false};

  rclcpp::Publisher<uav_interfaces::msg::UavStatus>::SharedPtr
    uav_status_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::GcsStatus>::SharedPtr
    gcs_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gcs_link_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr task_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::CarStatus>::SharedPtr
    car_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr car_link_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    pose_valid_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::FlightStatus>::SharedPtr
    flight_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    camera_link_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    visual_control_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_subscription_;

  rclcpp::TimerBase::SharedPtr io_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace gcs_serial_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gcs_serial_bridge::GcsSerialBridgeNode>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "gcs_serial_bridge启动失败：%s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
