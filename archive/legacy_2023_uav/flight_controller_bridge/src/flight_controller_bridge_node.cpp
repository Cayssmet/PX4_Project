#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "uav_interfaces/msg/flight_command.hpp"
#include "uav_interfaces/msg/flight_directive.hpp"
#include "uav_interfaces/msg/flight_status.hpp"
#include "uav_interfaces/msg/mission_status.hpp"
#include "uav_interfaces/msg/waypoint.hpp"

#include "flight_controller_bridge/coordinates.hpp"
#include "flight_controller_bridge/protocol.hpp"

namespace flight_controller_bridge
{
namespace
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

class SerialPort
{
public:
  SerialPort() = default;
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  ~SerialPort() {close_port();}

  bool open_port(const std::string & device, const int baud_rate, std::string & error)
  {
    close_port();
    speed_t speed{};
    if (!baud_to_termios(baud_rate, speed)) {
      error = "不支持的波特率: " + std::to_string(baud_rate);
      return false;
    }
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      error = std::string("打开 ") + device + " 失败: " + std::strerror(errno);
      return false;
    }
    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
      error = std::string("读取串口属性失败: ") + std::strerror(errno);
      close_port();
      return false;
    }
    cfmakeraw(&options);
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    options.c_cflag &= static_cast<tcflag_t>(~PARENB);
    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    options.c_cflag |= CS8;
#ifdef CRTSCTS
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      error = std::string("设置串口属性失败: ") + std::strerror(errno);
      close_port();
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    const int flags = fcntl(fd_, F_GETFL, 0);
    if (flags >= 0) {
      static_cast<void>(fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK));
    }
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const {return fd_ >= 0;}

  template<typename ByteCallback>
  bool read_available(ByteCallback && callback, std::string & error)
  {
    std::array<std::uint8_t, 256U> buffer{};
    while (true) {
      const ssize_t count = ::read(fd_, buffer.data(), buffer.size());
      if (count > 0) {
        for (ssize_t index = 0; index < count; ++index) {
          callback(buffer[static_cast<std::size_t>(index)]);
        }
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        return true;
      }
      if (errno == EINTR) {
        continue;
      }
      error = std::string("读取串口失败: ") + std::strerror(errno);
      return false;
    }
  }

  bool write_all(const std::uint8_t * data, const std::size_t length, std::string & error)
  {
    std::size_t written = 0U;
    while (written < length) {
      const ssize_t result = ::write(fd_, data + written, length - written);
      if (result > 0) {
        written += static_cast<std::size_t>(result);
        continue;
      }
      if (result < 0 && errno == EINTR) {
        continue;
      }
      error = std::string("写入串口失败: ") +
        (result == 0 ? "返回 0 字节" : std::strerror(errno));
      return false;
    }
    return true;
  }

private:
  static bool baud_to_termios(const int baud, speed_t & speed)
  {
    switch (baud) {
      case 9600: speed = B9600; return true;
      case 19200: speed = B19200; return true;
      case 38400: speed = B38400; return true;
      case 57600: speed = B57600; return true;
      case 115200: speed = B115200; return true;
#ifdef B230400
      case 230400: speed = B230400; return true;
#endif
      default: return false;
    }
  }

  int fd_{-1};
};

bool finite_twist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

bool quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q, double & yaw)
{
  if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
    !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    return false;
  }
  const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  const double x = q.x * inverse_norm;
  const double y = q.y * inverse_norm;
  const double z = q.z * inverse_norm;
  const double w = q.w * inverse_norm;
  yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(yaw);
}

bool to_i16_milli(const double value, std::int16_t & result)
{
  if (!std::isfinite(value)) {
    return false;
  }
  const double scaled = std::round(value * 1000.0);
  if (scaled < static_cast<double>(std::numeric_limits<std::int16_t>::min()) ||
    scaled > static_cast<double>(std::numeric_limits<std::int16_t>::max()))
  {
    return false;
  }
  result = static_cast<std::int16_t>(scaled);
  return true;
}

bool to_u16_milli(const double value, std::uint16_t & result)
{
  if (!std::isfinite(value)) {
    return false;
  }
  const double scaled = std::round(value * 1000.0);
  if (scaled < 0.0 || scaled > static_cast<double>(std::numeric_limits<std::uint16_t>::max())) {
    return false;
  }
  result = static_cast<std::uint16_t>(scaled);
  return true;
}

bool valid_v2_mode(const std::uint8_t mode)
{
  return mode <= static_cast<std::uint8_t>(V2ControlMode::kResetMission);
}

bool v2_mode_uses_height(const V2ControlMode mode)
{
  return mode == V2ControlMode::kTakeoff || mode == V2ControlMode::kHeightHold ||
         mode == V2ControlMode::kPrecisionDescent;
}

bool v2_mode_is_continuous(const V2ControlMode mode)
{
  return mode == V2ControlMode::kStandby || mode == V2ControlMode::kHeightHold ||
         mode == V2ControlMode::kPrecisionDescent;
}

const char * v2_error_code_name(const V2ErrorCode code)
{
  switch (code) {
    case V2ErrorCode::kOk: return "OK";
    case V2ErrorCode::kRcFailsafe: return "RC_FAILSAFE";
    case V2ErrorCode::kCommandTimeout: return "COMMAND_TIMEOUT";
    case V2ErrorCode::kHeightTimeout: return "HEIGHT_TIMEOUT";
    case V2ErrorCode::kHeightOutOfRange: return "HEIGHT_OUT_OF_RANGE";
    case V2ErrorCode::kCrcOrFrameError: return "FRAME_ERROR";
    case V2ErrorCode::kSequenceError: return "SEQUENCE_ERROR";
    case V2ErrorCode::kControlNotAuthorized: return "NOT_AUTHORIZED";
    case V2ErrorCode::kInvalidMode: return "INVALID_MODE";
    case V2ErrorCode::kInvalidSession: return "INVALID_SESSION";
    case V2ErrorCode::kDuplicateAction: return "DUPLICATE_ACTION";
    case V2ErrorCode::kActionNotAllowed: return "ACTION_NOT_ALLOWED";
    case V2ErrorCode::kTakeoffTimeout: return "TAKEOFF_TIMEOUT";
    case V2ErrorCode::kLandTimeout: return "LAND_TIMEOUT";
    case V2ErrorCode::kResetNotAllowed: return "RESET_NOT_ALLOWED";
    case V2ErrorCode::kInternalError: return "INTERNAL_ERROR";
    case V2ErrorCode::kInvalidField: return "INVALID_FIELD";
    case V2ErrorCode::kOldAction: return "OLD_ACTION";
  }
  return "UNKNOWN_ERROR";
}

}  // namespace

class FlightControllerBridgeNode : public rclcpp::Node
{
public:
  FlightControllerBridgeNode()
  : Node("flight_controller_bridge")
  {
    declare_and_load_parameters();
    validate_parameters();

    command_publisher_ = create_publisher<uav_interfaces::msg::FlightCommand>(
      "/flight/command", rclcpp::QoS(10).reliable());
    status_publisher_ = create_publisher<uav_interfaces::msg::FlightStatus>(
      "/flight/status", rclcpp::QoS(10).reliable());
    const auto normal_qos = rclcpp::QoS(10).reliable();
    const auto retained_qos = rclcpp::QoS(1).reliable().transient_local();

    pid_command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/pid/cmd_vel", normal_qos,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        latest_pid_command_ = *message;
        pid_command_received_ = true;
        pid_command_time_ = SteadyClock::now();
      });
    pid_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/pid/control_valid", normal_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        pid_valid_ = message->data;
        pid_valid_received_ = true;
        pid_valid_time_ = SteadyClock::now();
      });
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/map/current_pose", normal_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        latest_pose_ = *message;
        pose_received_ = true;
        pose_time_ = SteadyClock::now();
      });
    pose_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/map/current_pose_valid", normal_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        pose_valid_ = message->data;
        pose_valid_received_ = true;
        pose_valid_time_ = SteadyClock::now();
      });
    waypoint_subscription_ = create_subscription<uav_interfaces::msg::Waypoint>(
      "/waypoint/current", retained_qos,
      [this](const uav_interfaces::msg::Waypoint::SharedPtr message) {
        const bool changed = !waypoint_received_ ||
          message->id != latest_waypoint_.id ||
          message->target_height != latest_waypoint_.target_height;
        latest_waypoint_ = *message;
        waypoint_received_ = true;
        if (changed) {
          arm_new_target_guard();
        }
      });
    mission_subscription_ = create_subscription<uav_interfaces::msg::MissionStatus>(
      "/mission/status", retained_qos,
      [this](const uav_interfaces::msg::MissionStatus::SharedPtr message) {
        const bool changed = !mission_received_ ||
          message->route_id != latest_mission_.route_id ||
          message->current_waypoint_id != latest_mission_.current_waypoint_id;
        latest_mission_ = *message;
        mission_received_ = true;
        mission_time_ = SteadyClock::now();
        if (changed) {
          arm_new_target_guard();
        }
      });
    land_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/mission/land_request", retained_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        latest_land_request_ = message->data;
        land_request_received_ = true;
        land_request_time_ = SteadyClock::now();
      });
    directive_subscription_ = create_subscription<uav_interfaces::msg::FlightDirective>(
      "/flight/directive", retained_qos,
      [this](const uav_interfaces::msg::FlightDirective::SharedPtr message) {
        const bool changed = !directive_received_ ||
          message->control_mode != latest_directive_.control_mode ||
          message->session_id != latest_directive_.session_id ||
          message->action_id != latest_directive_.action_id ||
          message->target_height != latest_directive_.target_height;
        latest_directive_ = *message;
        directive_received_ = true;
        directive_time_ = SteadyClock::now();
        if (changed &&
          (message->control_mode == static_cast<std::uint8_t>(V2ControlMode::kHeightHold) ||
          message->control_mode == static_cast<std::uint8_t>(V2ControlMode::kPrecisionDescent)))
        {
          arm_new_target_guard();
        }
      });

    const auto period = std::chrono::duration<double>(1.0 / send_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FlightControllerBridgeNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "飞控桥已启动：协议=V%d，串口=%s，波特率=%d，频率=%.1f Hz；默认只发无效零速帧。",
      protocol_version_, serial_device_.c_str(), baud_rate_, send_rate_hz_);
  }

private:
  void declare_and_load_parameters()
  {
    protocol_version_ = declare_parameter<int>("protocol_version", 1);
    serial_device_ = declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 50.0);
    reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 1000);
    flight_feedback_timeout_ms_ = declare_parameter<int>("flight_feedback_timeout_ms", 300);
    max_ack_lag_frames_ = declare_parameter<int>("max_ack_lag_frames", 10);
    require_flight_feedback_before_control_ =
      declare_parameter<bool>("require_flight_feedback_before_control", true);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    pid_command_timeout_ms_ = declare_parameter<int>("pid_command_timeout_ms", 300);
    new_target_pid_guard_ms_ = declare_parameter<int>("new_target_pid_guard_ms", 100);
    pose_timeout_ms_ = declare_parameter<int>("pose_timeout_ms", 300);
    mission_status_timeout_ms_ = declare_parameter<int>("mission_status_timeout_ms", 500);
    land_request_timeout_ms_ = declare_parameter<int>("land_request_timeout_ms", 500);
    directive_timeout_ms_ = declare_parameter<int>("directive_timeout_ms", 500);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 1.5);
    max_vertical_speed_ = declare_parameter<double>("max_vertical_speed", 0.5);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 2.0);
    min_target_height_ = declare_parameter<double>("min_target_height", 0.1);
    max_target_height_ = declare_parameter<double>("max_target_height", 1.7);
    print_status_ = declare_parameter<bool>("print_status", true);
  }

  void validate_parameters() const
  {
    if (protocol_version_ != 1 && protocol_version_ != 2) {
      throw std::invalid_argument("protocol_version只能为1或2");
    }
    if (serial_device_.empty()) {
      throw std::invalid_argument("serial_device不能为空");
    }
    if (!std::isfinite(send_rate_hz_) || send_rate_hz_ < 1.0 || send_rate_hz_ > 200.0) {
      throw std::invalid_argument("send_rate_hz必须在[1,200] Hz内");
    }
    if (reconnect_interval_ms_ <= 0 || flight_feedback_timeout_ms_ <= 0 ||
      pid_command_timeout_ms_ <= 0 || pose_timeout_ms_ <= 0 ||
      mission_status_timeout_ms_ <= 0 || land_request_timeout_ms_ <= 0 ||
      directive_timeout_ms_ <= 0)
    {
      throw std::invalid_argument("全部超时和重连时间必须大于0 ms");
    }
    if (max_ack_lag_frames_ < 0 || max_ack_lag_frames_ > 127) {
      throw std::invalid_argument("max_ack_lag_frames必须在[0,127]内");
    }
    if (new_target_pid_guard_ms_ < 0 || new_target_pid_guard_ms_ > 1000) {
      throw std::invalid_argument("new_target_pid_guard_ms必须在[0,1000] ms内");
    }
    if (!std::isfinite(max_linear_speed_) || max_linear_speed_ <= 0.0 ||
      !std::isfinite(max_vertical_speed_) || max_vertical_speed_ <= 0.0 ||
      !std::isfinite(max_yaw_rate_) || max_yaw_rate_ <= 0.0)
    {
      throw std::invalid_argument("速度安全上限必须为正数");
    }
    if (!std::isfinite(min_target_height_) || !std::isfinite(max_target_height_) ||
      min_target_height_ < kV2ContinuousMinHeightM ||
      max_target_height_ < min_target_height_ ||
      max_target_height_ > kV2MaxTargetHeightM)
    {
      throw std::invalid_argument("目标高度上下限非法");
    }
  }

  static bool fresh(
    const bool received, const SteadyTime & received_time,
    const SteadyTime & now, const int timeout_ms)
  {
    return received && now - received_time <= std::chrono::milliseconds(timeout_ms);
  }

  void arm_new_target_guard()
  {
    pid_command_received_ = false;
    pid_valid_received_ = false;
    target_switch_time_ = SteadyClock::now();
    target_guard_active_ = true;
  }

  void reset_link_state()
  {
    v1_parser_.reset();
    v2_parser_.reset();
    feedback_received_ = false;
    sent_sequence_available_ = false;
  }

  void ensure_serial_open(const SteadyTime & now)
  {
    if (serial_.is_open()) {
      return;
    }
    if (open_attempted_ &&
      now - last_open_attempt_ < std::chrono::milliseconds(reconnect_interval_ms_))
    {
      return;
    }
    open_attempted_ = true;
    last_open_attempt_ = now;
    std::string error;
    if (serial_.open_port(serial_device_, baud_rate_, error)) {
      reset_link_state();
      RCLCPP_INFO(get_logger(), "飞控串口已连接：%s（V%d）", serial_device_.c_str(), protocol_version_);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "飞控串口尚未连接：%s；节点将继续重试。", error.c_str());
    }
  }

  void close_serial_after_error(const std::string & error)
  {
    RCLCPP_ERROR(get_logger(), "%s；关闭串口并等待重连。", error.c_str());
    serial_.close_port();
    reset_link_state();
  }

  void accept_acknowledged_status(
    const std::uint8_t acknowledged_sequence,
    const SteadyTime & now,
    const std::function<void()> & store_status)
  {
    if (!sent_sequence_available_ ||
      !acknowledgement_is_recent(
        last_sent_sequence_, acknowledged_sequence,
        static_cast<std::uint8_t>(max_ack_lag_frames_)))
    {
      ++rejected_ack_frame_count_;
      return;
    }
    store_status();
    feedback_received_ = true;
    feedback_time_ = now;
    ++valid_status_frame_count_;
  }

  void read_flight_status(const SteadyTime & now)
  {
    if (!serial_.is_open()) {
      return;
    }
    std::string error;
    const bool ok = serial_.read_available(
      [this, &now](const std::uint8_t byte) {
        if (protocol_version_ == 1) {
          if (const auto parsed = v1_parser_.push(byte); parsed.has_value()) {
            accept_acknowledged_status(
              parsed->acknowledged_sequence, now,
              [this, parsed]() {latest_v1_status_ = *parsed;});
          }
        } else if (const auto parsed = v2_parser_.push(byte); parsed.has_value()) {
          accept_acknowledged_status(
            parsed->acknowledged_sequence, now,
            [this, parsed]() {latest_v2_status_ = *parsed;});
        }
      }, error);
    if (!ok) {
      close_serial_after_error(error);
    }
  }

  bool feedback_is_fresh(const SteadyTime & now) const
  {
    return serial_.is_open() &&
      fresh(feedback_received_, feedback_time_, now, flight_feedback_timeout_ms_);
  }

  bool feedback_failsafe() const
  {
    return protocol_version_ == 1 ? latest_v1_status_.failsafe : latest_v2_status_.failsafe;
  }

  void update_v1_land_latch(const SteadyTime & now)
  {
    if (protocol_version_ != 1 ||
      !fresh(mission_received_, mission_time_, now, mission_status_timeout_ms_) ||
      !fresh(land_request_received_, land_request_time_, now, land_request_timeout_ms_))
    {
      return;
    }
    if (known_route_id_ != latest_mission_.route_id) {
      known_route_id_ = latest_mission_.route_id;
      land_latched_ = false;
      land_completed_latched_ = false;
      land_latched_time_ = SteadyTime{};
    }
    if (latest_land_request_ && latest_mission_.route_valid && latest_mission_.mission_completed) {
      if (!land_latched_) {
        land_latched_ = true;
        land_latched_time_ = now;
      }
    }
    if (land_latched_ && feedback_is_fresh(now) &&
      feedback_time_ > land_latched_time_ && latest_v1_status_.land_completed)
    {
      land_completed_latched_ = true;
    }
  }

  void initialize_ros_command(uav_interfaces::msg::FlightCommand & command) const
  {
    command.header.stamp = get_clock()->now();
    command.header.frame_id = body_frame_;
    command.protocol_version = static_cast<std::uint8_t>(protocol_version_);
    command.control_mode = static_cast<std::uint8_t>(V2ControlMode::kStandby);
    command.session_id = 0U;
    command.action_id = 0U;
    command.velocity_x = 0.0F;
    command.velocity_y = 0.0F;
    command.velocity_z = 0.0F;
    command.yaw_rate = 0.0F;
    command.target_height = 0.0F;
    command.control_valid = false;
    command.land_request = false;
  }

  bool fill_v1_motion(
    const SteadyTime & now,
    WireCommand & wire,
    uav_interfaces::msg::FlightCommand & ros_command,
    std::string & reason)
  {
    if (!fresh(mission_received_, mission_time_, now, mission_status_timeout_ms_)) {
      reason = "任务状态缺失或过期";
      return false;
    }
    if (!latest_mission_.route_valid || !latest_mission_.target_sent ||
      latest_mission_.mission_completed)
    {
      reason = "没有正在跟踪的合法航点";
      return false;
    }
    if (!waypoint_received_ || latest_waypoint_.id != latest_mission_.current_waypoint_id) {
      reason = "当前航点缺失或id不一致";
      return false;
    }
    if (!std::isfinite(latest_waypoint_.target_height) ||
      latest_waypoint_.target_height < min_target_height_ ||
      latest_waypoint_.target_height > max_target_height_ ||
      !to_u16_milli(latest_waypoint_.target_height, wire.target_height_mm))
    {
      reason = "目标高度非法或越界";
      return false;
    }
    ros_command.target_height = static_cast<float>(latest_waypoint_.target_height);
    if (!fill_horizontal_motion(now, false, wire.velocity_x_mm_s, wire.velocity_y_mm_s,
      wire.yaw_rate_mrad_s, ros_command, reason))
    {
      return false;
    }
    return true;
  }

  bool fill_horizontal_motion(
    const SteadyTime & now,
    const bool allow_body_frame,
    std::int16_t & velocity_x_mm_s,
    std::int16_t & velocity_y_mm_s,
    std::int16_t & yaw_rate_mrad_s,
    uav_interfaces::msg::FlightCommand & ros_command,
    std::string & reason)
  {
    if (target_guard_active_) {
      const SteadyTime not_before = target_switch_time_ +
        std::chrono::milliseconds(new_target_pid_guard_ms_);
      if (now < not_before || !pid_command_received_ || !pid_valid_received_ ||
        pid_command_time_ < not_before || pid_valid_time_ < not_before)
      {
        reason = "等待新目标对应的PID输出";
        return false;
      }
      target_guard_active_ = false;
    }
    if (!fresh(pid_command_received_, pid_command_time_, now, pid_command_timeout_ms_) ||
      !fresh(pid_valid_received_, pid_valid_time_, now, pid_command_timeout_ms_) || !pid_valid_)
    {
      reason = "PID命令缺失、过期或无效";
      return false;
    }
    if (!finite_twist(latest_pid_command_.twist)) {
      reason = "PID命令含非法数值";
      return false;
    }
    const double input_x = latest_pid_command_.twist.linear.x;
    const double input_y = latest_pid_command_.twist.linear.y;
    const double yaw_rate = latest_pid_command_.twist.angular.z;
    if (std::hypot(input_x, input_y) > max_linear_speed_ || std::abs(yaw_rate) > max_yaw_rate_) {
      reason = "PID速度超过飞控桥安全范围";
      return false;
    }

    BodyVelocity body{};
    if (latest_pid_command_.header.frame_id == body_frame_ && allow_body_frame) {
      body.x = input_x;
      body.y = input_y;
    } else {
      if (latest_pid_command_.header.frame_id != map_frame_) {
        reason = "PID命令frame非法";
        return false;
      }
      if (!fresh(pose_received_, pose_time_, now, pose_timeout_ms_) ||
        !fresh(pose_valid_received_, pose_valid_time_, now, pose_timeout_ms_) || !pose_valid_ ||
        latest_pose_.header.frame_id != map_frame_)
      {
        reason = "当前位置缺失、过期或无效";
        return false;
      }
      double yaw = 0.0;
      if (!quaternion_to_yaw(latest_pose_.pose.orientation, yaw)) {
        reason = "当前位置四元数非法";
        return false;
      }
      body = map_velocity_to_body(input_x, input_y, yaw);
    }
    if (!to_i16_milli(body.x, velocity_x_mm_s) ||
      !to_i16_milli(body.y, velocity_y_mm_s) ||
      !to_i16_milli(yaw_rate, yaw_rate_mrad_s))
    {
      reason = "定点数换算溢出";
      return false;
    }
    ros_command.velocity_x = static_cast<float>(body.x);
    ros_command.velocity_y = static_cast<float>(body.y);
    ros_command.yaw_rate = static_cast<float>(yaw_rate);
    return true;
  }

  WireCommand build_v1_command(
    const SteadyTime & now,
    uav_interfaces::msg::FlightCommand & ros_command,
    std::string & reason)
  {
    WireCommand wire;
    wire.sequence = next_sequence_;
    initialize_ros_command(ros_command);
    if (waypoint_received_ && std::isfinite(latest_waypoint_.target_height)) {
      ros_command.target_height = static_cast<float>(latest_waypoint_.target_height);
      static_cast<void>(to_u16_milli(latest_waypoint_.target_height, wire.target_height_mm));
    }
    if (!serial_.is_open()) {
      reason = "串口未连接";
      return wire;
    }
    if (land_latched_ && !land_completed_latched_) {
      wire.control_valid = true;
      wire.land_request = true;
      ros_command.control_valid = true;
      ros_command.land_request = true;
      ros_command.control_mode = static_cast<std::uint8_t>(V2ControlMode::kFinalLand);
      reason = "V1执行降落";
      return wire;
    }
    if (require_flight_feedback_before_control_ && !feedback_is_fresh(now)) {
      reason = "尚无新鲜飞控反馈";
      return wire;
    }
    if (feedback_is_fresh(now) && feedback_failsafe()) {
      reason = "飞控报告failsafe";
      return wire;
    }
    if (!fill_v1_motion(now, wire, ros_command, reason)) {
      return wire;
    }
    wire.control_valid = true;
    ros_command.control_valid = true;
    ros_command.control_mode = static_cast<std::uint8_t>(V2ControlMode::kHeightHold);
    reason = "V1普通控制有效";
    return wire;
  }

  V2WireCommand build_v2_command(
    const SteadyTime & now,
    uav_interfaces::msg::FlightCommand & ros_command,
    std::string & reason)
  {
    V2WireCommand wire;
    wire.sequence = next_sequence_;
    initialize_ros_command(ros_command);
    if (!serial_.is_open()) {
      reason = "串口未连接";
      return wire;
    }
    if (!fresh(directive_received_, directive_time_, now, directive_timeout_ms_)) {
      reason = "V2指令缺失或过期";
      return wire;
    }
    if (!valid_v2_mode(latest_directive_.control_mode)) {
      reason = "V2模式非法";
      return wire;
    }
    const auto mode = static_cast<V2ControlMode>(latest_directive_.control_mode);
    wire.control_mode = mode;
    wire.session_id = latest_directive_.session_id;
    wire.action_id = latest_directive_.action_id;
    ros_command.control_mode = latest_directive_.control_mode;
    ros_command.session_id = latest_directive_.session_id;
    ros_command.action_id = latest_directive_.action_id;
    ros_command.target_height = latest_directive_.target_height;
    if (!latest_directive_.control_valid) {
      reason = "V2 control_valid=false";
      return wire;
    }
    if (wire.session_id == 0U) {
      reason = "V2 session_id不能为0";
      return wire;
    }
    if ((v2_mode_is_continuous(mode) && wire.action_id != 0U) ||
      (!v2_mode_is_continuous(mode) && wire.action_id == 0U))
    {
      reason = "V2 action_id与模式不匹配";
      return wire;
    }
    if (v2_mode_uses_height(mode)) {
      if (!std::isfinite(latest_directive_.target_height) ||
        !v2_target_height_is_valid(mode, latest_directive_.target_height) ||
        latest_directive_.target_height < min_target_height_ ||
        latest_directive_.target_height > max_target_height_ ||
        !to_u16_milli(latest_directive_.target_height, wire.target_height_mm))
      {
        reason = "V2目标高度非法或越界";
        return wire;
      }
    } else {
      ros_command.target_height = 0.0F;
    }

    if (mode == V2ControlMode::kStandby) {
      wire.control_valid = true;
      ros_command.control_valid = true;
      reason = "V2 STANDBY/session握手";
      return wire;
    }
    if (!feedback_is_fresh(now)) {
      reason = "V2尚无新鲜飞控反馈";
      return wire;
    }
    if (latest_v2_status_.failsafe) {
      reason = "V2飞控报告failsafe";
      return wire;
    }
    if (latest_v2_status_.active_session_id != wire.session_id) {
      reason = "V2 session尚未回显确认";
      return wire;
    }

    if (mode == V2ControlMode::kTakeoff || mode == V2ControlMode::kFinalLand ||
      mode == V2ControlMode::kResetMission)
    {
      wire.control_valid = true;
      ros_command.control_valid = true;
      ros_command.land_request = mode == V2ControlMode::kFinalLand;
      reason = "V2离散动作有效";
      return wire;
    }

    std::string horizontal_reason;
    if (!fill_horizontal_motion(
        now, true, wire.velocity_x_mm_s, wire.velocity_y_mm_s,
        wire.yaw_rate_mrad_s, ros_command, horizontal_reason))
    {
      if (mode == V2ControlMode::kHeightHold) {
        wire.velocity_x_mm_s = 0;
        wire.velocity_y_mm_s = 0;
        wire.yaw_rate_mrad_s = 0;
        wire.control_valid = true;
        ros_command.velocity_x = 0.0F;
        ros_command.velocity_y = 0.0F;
        ros_command.yaw_rate = 0.0F;
        ros_command.control_valid = true;
        reason = "V2定高有效，定位/PID无效时水平零速：" + horizontal_reason;
        return wire;
      }
      reason = horizontal_reason;
      return wire;
    }
    if (mode == V2ControlMode::kPrecisionDescent) {
      if (!std::isfinite(latest_directive_.vertical_velocity) ||
        std::abs(latest_directive_.vertical_velocity) > max_vertical_speed_ ||
        !to_i16_milli(latest_directive_.vertical_velocity, wire.velocity_z_mm_s))
      {
        reason = "V2垂直速度非法或越界";
        return wire;
      }
      ros_command.velocity_z = latest_directive_.vertical_velocity;
    }
    wire.control_valid = true;
    ros_command.control_valid = true;
    reason = mode == V2ControlMode::kPrecisionDescent ?
      "V2精密下降有效" : "V2定高控制有效";
    return wire;
  }

  void publish_flight_status(const SteadyTime & now)
  {
    uav_interfaces::msg::FlightStatus status;
    status.header.stamp = get_clock()->now();
    status.protocol_version = static_cast<std::uint8_t>(protocol_version_);
    const bool communication_ok = feedback_is_fresh(now);
    status.communication_ok = communication_ok;
    if (protocol_version_ == 1) {
      status.height_reached = communication_ok && latest_v1_status_.height_reached;
      status.land_accepted = communication_ok && latest_v1_status_.land_accepted;
      status.land_completed = communication_ok && latest_v1_status_.land_completed;
      status.failsafe = communication_ok && latest_v1_status_.failsafe;
    } else {
      status.current_mode = static_cast<std::uint8_t>(latest_v2_status_.current_mode);
      status.error_code = static_cast<std::uint8_t>(latest_v2_status_.error_code);
      status.active_session_id = latest_v2_status_.active_session_id;
      status.active_action_id = latest_v2_status_.active_action_id;
      status.current_height = latest_v2_status_.height_valid ?
        static_cast<float>(latest_v2_status_.current_height_mm) / 1000.0F : 0.0F;
      status.vertical_velocity =
        static_cast<float>(latest_v2_status_.vertical_velocity_mm_s) / 1000.0F;
      status.height_age_ms = latest_v2_status_.height_age_ms;
      status.command_age_ms = latest_v2_status_.command_age_ms;
      status.action_result = static_cast<std::uint8_t>(latest_v2_status_.action_result);
      status.height_valid = communication_ok && latest_v2_status_.height_valid;
      status.height_reached = communication_ok && latest_v2_status_.height_reached;
      status.armed = communication_ok && latest_v2_status_.armed;
      status.airborne = communication_ok && latest_v2_status_.airborne;
      status.landed = communication_ok && latest_v2_status_.landed;
      status.action_accepted = communication_ok && latest_v2_status_.action_accepted;
      status.action_completed = communication_ok && latest_v2_status_.action_completed;
      status.failsafe = communication_ok && latest_v2_status_.failsafe;
      status.land_accepted = status.action_accepted &&
        latest_v2_status_.current_mode == V2ControlMode::kFinalLand;
      status.land_completed = status.landed && status.action_completed;
    }
    status_publisher_->publish(status);
  }

  void timer_callback()
  {
    const SteadyTime now = SteadyClock::now();
    ensure_serial_open(now);
    read_flight_status(now);
    update_v1_land_latch(now);

    uav_interfaces::msg::FlightCommand ros_command;
    std::string reason;
    bool write_ok = false;
    std::string error;
    if (protocol_version_ == 1) {
      const WireCommand wire = build_v1_command(now, ros_command, reason);
      command_publisher_->publish(ros_command);
      if (serial_.is_open()) {
        const auto frame = encode_command_frame(wire);
        write_ok = serial_.write_all(frame.data(), frame.size(), error);
      }
    } else {
      const V2WireCommand wire = build_v2_command(now, ros_command, reason);
      command_publisher_->publish(ros_command);
      if (serial_.is_open()) {
        const auto frame = encode_v2_command_frame(wire);
        write_ok = serial_.write_all(frame.data(), frame.size(), error);
      }
    }
    if (serial_.is_open()) {
      if (write_ok) {
        ++sent_frame_count_;
        last_sent_sequence_ = next_sequence_;
        sent_sequence_available_ = true;
        next_sequence_ = static_cast<std::uint8_t>(next_sequence_ + 1U);
      } else {
        close_serial_after_error(error);
        reason = "串口写入失败";
      }
    }
    publish_flight_status(now);

    if (print_status_) {
      const auto displayed_error = protocol_version_ == 2 ?
        latest_v2_status_.error_code : V2ErrorCode::kOk;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "V%d 串口=%s 反馈=%s 安全门=%s；mode=%u session=%u action=%u "
        "vx=%.3f vy=%.3f vz=%.3f yaw=%.3f h=%.3f valid=%s；error=%s(0x%02X)；"
        "TX=%llu RX=%llu ACK拒绝=%llu",
        protocol_version_, serial_.is_open() ? "已连接" : "未连接",
        feedback_is_fresh(now) ? "正常" : "无/过期", reason.c_str(),
        ros_command.control_mode, ros_command.session_id, ros_command.action_id,
        ros_command.velocity_x, ros_command.velocity_y, ros_command.velocity_z,
        ros_command.yaw_rate, ros_command.target_height,
        ros_command.control_valid ? "true" : "false",
        v2_error_code_name(displayed_error), static_cast<unsigned int>(displayed_error),
        static_cast<unsigned long long>(sent_frame_count_),
        static_cast<unsigned long long>(valid_status_frame_count_),
        static_cast<unsigned long long>(rejected_ack_frame_count_));
    }
  }

  int protocol_version_{1};
  std::string serial_device_;
  int baud_rate_{115200};
  double send_rate_hz_{50.0};
  int reconnect_interval_ms_{1000};
  int flight_feedback_timeout_ms_{300};
  int max_ack_lag_frames_{10};
  bool require_flight_feedback_before_control_{true};
  std::string map_frame_{"map"};
  std::string body_frame_{"base_link"};
  int pid_command_timeout_ms_{300};
  int new_target_pid_guard_ms_{100};
  int pose_timeout_ms_{300};
  int mission_status_timeout_ms_{500};
  int land_request_timeout_ms_{500};
  int directive_timeout_ms_{500};
  double max_linear_speed_{1.5};
  double max_vertical_speed_{0.5};
  double max_yaw_rate_{2.0};
  double min_target_height_{0.1};
  double max_target_height_{1.7};
  bool print_status_{true};

  geometry_msgs::msg::TwistStamped latest_pid_command_;
  bool pid_command_received_{false};
  SteadyTime pid_command_time_{};
  bool pid_valid_{false};
  bool pid_valid_received_{false};
  SteadyTime pid_valid_time_{};
  bool target_guard_active_{false};
  SteadyTime target_switch_time_{};

  geometry_msgs::msg::PoseStamped latest_pose_;
  bool pose_received_{false};
  SteadyTime pose_time_{};
  bool pose_valid_{false};
  bool pose_valid_received_{false};
  SteadyTime pose_valid_time_{};

  uav_interfaces::msg::Waypoint latest_waypoint_;
  bool waypoint_received_{false};
  uav_interfaces::msg::MissionStatus latest_mission_;
  bool mission_received_{false};
  SteadyTime mission_time_{};
  bool latest_land_request_{false};
  bool land_request_received_{false};
  SteadyTime land_request_time_{};
  uav_interfaces::msg::FlightDirective latest_directive_;
  bool directive_received_{false};
  SteadyTime directive_time_{};

  SerialPort serial_;
  StatusStreamParser v1_parser_;
  V2StatusStreamParser v2_parser_;
  bool open_attempted_{false};
  SteadyTime last_open_attempt_{};
  WireStatus latest_v1_status_;
  V2WireStatus latest_v2_status_;
  bool feedback_received_{false};
  SteadyTime feedback_time_{};
  std::uint8_t next_sequence_{0U};
  std::uint8_t last_sent_sequence_{0U};
  bool sent_sequence_available_{false};
  std::uint64_t sent_frame_count_{0U};
  std::uint64_t valid_status_frame_count_{0U};
  std::uint64_t rejected_ack_frame_count_{0U};

  std::uint32_t known_route_id_{0U};
  bool land_latched_{false};
  bool land_completed_latched_{false};
  SteadyTime land_latched_time_{};

  rclcpp::Publisher<uav_interfaces::msg::FlightCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::FlightStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pid_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pid_valid_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pose_valid_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::Waypoint>::SharedPtr waypoint_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::MissionStatus>::SharedPtr mission_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_subscription_;
  rclcpp::Subscription<uav_interfaces::msg::FlightDirective>::SharedPtr directive_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace flight_controller_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<flight_controller_bridge::FlightControllerBridgeNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("flight_controller_bridge"),
      "飞控桥启动失败：%s", exception.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
