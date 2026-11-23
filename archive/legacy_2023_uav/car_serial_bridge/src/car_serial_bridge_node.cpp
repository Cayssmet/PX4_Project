#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "uav_interfaces/msg/car_status.hpp"
#include "uav_interfaces/msg/flight_status.hpp"
#include "uav_interfaces/msg/mission_sync.hpp"
#include "car_serial_bridge/protocol.hpp"

namespace car_serial_bridge
{
namespace
{
constexpr std::uint8_t kReady = 0x00U;
constexpr std::uint8_t kCountdown = 0x01U;
constexpr std::uint8_t kMissionStart = 0x02U;
constexpr std::uint8_t kTakeoffHover = 0x03U;
constexpr std::uint8_t kRightShift = 0x04U;
constexpr std::uint8_t kForwardChase = 0x05U;
constexpr std::uint8_t kFollowCar = 0x07U;
constexpr std::uint8_t kTask1AlignDrop = 0x10U;
constexpr std::uint8_t kTask1Drop = 0x11U;
constexpr std::uint8_t kTask1ReturnH = 0x12U;
constexpr std::uint8_t kTask1LandH = 0x13U;
constexpr std::uint8_t kTask1Complete = 0x14U;
constexpr std::uint8_t kTask2AlignLand = 0x20U;
constexpr std::uint8_t kTask2Descend = 0x21U;
constexpr std::uint8_t kTask2OnCar = 0x22U;
constexpr std::uint8_t kTask2Hold5S = 0x23U;
constexpr std::uint8_t kTask2Retakeoff = 0x24U;
constexpr std::uint8_t kTask2ReturnH = 0x25U;
constexpr std::uint8_t kTask2LandH = 0x26U;
constexpr std::uint8_t kTask2Complete = 0x27U;
constexpr std::uint8_t kError = 0xF1U;

std::uint8_t phase_code(const std::string & phase)
{
  if (phase == "WAIT_START") {return kReady;}
  if (phase == "WAIT_READY") {return kMissionStart;}
  if (phase == "TAKEOFF_HOVER") {return kTakeoffHover;}
  if (phase == "RIGHT_SHIFT") {return kRightShift;}
  if (phase == "FORWARD_CHASE") {return kForwardChase;}
  if (phase == "TASK1_FOLLOW") {return kFollowCar;}
  if (phase == "TASK1_ALIGN_DROP") {return kTask1AlignDrop;}
  if (phase == "TASK1_DROP") {return kTask1Drop;}
  if (phase == "TASK1_RETURN_HOME") {return kTask1ReturnH;}
  if (phase == "TASK1_LAND_HOME") {return kTask1LandH;}
  if (phase == "TASK1_COMPLETE") {return kTask1Complete;}
  if (phase == "TASK2_VISUAL_APPROACH" || phase == "TASK2_ALIGN_LAND") {return kTask2AlignLand;}
  if (phase == "TASK2_DESCEND") {return kTask2Descend;}
  if (phase == "TASK2_ON_CAR") {return kTask2OnCar;}
  if (phase == "TASK2_HOLD_5S") {return kTask2Hold5S;}
  if (phase == "TASK2_RETAKEOFF") {return kTask2Retakeoff;}
  if (phase == "TASK2_RETURN_H") {return kTask2ReturnH;}
  if (phase == "TASK2_LAND_H") {return kTask2LandH;}
  if (phase == "TASK2_COMPLETE") {return kTask2Complete;}
  if (phase == "ERROR") {return kError;}
  return kReady;
}

std::uint8_t uav_state_for(
  std::uint8_t phase, const uav_interfaces::msg::FlightStatus & flight, bool flight_fresh)
{
  if (flight_fresh && flight.failsafe) {return 0xF0U;}
  if (phase == kMissionStart || phase == kTakeoffHover) {
    return flight_fresh && flight.active_action_id == 1U &&
           flight.action_result == uav_interfaces::msg::FlightStatus::ACTION_SUCCEEDED ?
           0x02U : 0x01U;
  }
  if (phase == kRightShift || phase == kForwardChase) {return 0x03U;}
  if (phase == kFollowCar || phase == kTask1AlignDrop || phase == kTask1Drop) {return 0x04U;}
  if (phase == kTask2AlignLand || phase == kTask2Descend) {return 0x05U;}
  if (phase == kTask2OnCar || phase == kTask2Hold5S) {return 0x06U;}
  if (phase == kTask2Retakeoff) {return 0x07U;}
  if (phase == kTask2ReturnH || phase == kTask2LandH) {
    return flight_fresh && flight.landed ? 0x09U : 0x08U;
  }
  if (phase == kTask2Complete) {return 0x06U;}
  if (phase == kTask1ReturnH || phase == kTask1LandH) {
    return flight_fresh && flight.landed ? 0x09U : 0x08U;
  }
  if (phase == kTask1Complete) {return 0x09U;}
  if (phase == kError) {return 0xF1U;}
  return 0x00U;
}

speed_t baud_constant(int baud)
{
  if (baud == 115200) {return B115200;}
  throw std::invalid_argument("统一设备协议固定使用115200波特率");
}
}  // namespace

class CarSerialBridgeNode : public rclcpp::Node
{
public:
  CarSerialBridgeNode() : Node("car_serial_bridge")
  {
    device_ = declare_parameter<std::string>("serial_device", "/dev/car_bluetooth");
    baud_ = declare_parameter<int>("baud_rate", 115200);
    sync_rate_ = declare_parameter<double>("sync_rate_hz", 10.0);
    timeout_ms_ = declare_parameter<int>("status_timeout_ms", 500);
    reconnect_ms_ = declare_parameter<int>("reconnect_interval_ms", 1000);
    flight_timeout_ms_ = declare_parameter<int>("flight_status_timeout_ms", 500);
    print_status_ = declare_parameter<bool>("print_status", true);
    if (device_.empty() || sync_rate_ <= 0.0 || sync_rate_ > 20.0 ||
      timeout_ms_ <= 0 || reconnect_ms_ <= 0 || flight_timeout_ms_ <= 0)
    {
      throw std::invalid_argument("小车串口桥参数非法");
    }
    (void)baud_constant(baud_);

    const auto qos = rclcpp::QoS(10).reliable();
    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    status_pub_ = create_publisher<uav_interfaces::msg::CarStatus>("/car/status", retained);
    sync_pub_ = create_publisher<uav_interfaces::msg::MissionSync>("/car/mission_sync", retained);
    link_pub_ = create_publisher<std_msgs::msg::Bool>("/car/link_ok", retained);
    drop_window_pub_ = create_publisher<std_msgs::msg::Bool>("/car/drop_window", qos);
    lap_complete_pub_ = create_publisher<std_msgs::msg::Bool>("/car/lap_complete", retained);
    task_pub_ = create_publisher<std_msgs::msg::UInt8>("/competition/task_mode", retained);
    start_pub_ = create_publisher<std_msgs::msg::Bool>("/competition/start", qos);

    phase_sub_ = create_subscription<std_msgs::msg::String>(
      "/competition/phase", retained, [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "SAFE_HOLD") {return;}
        if (phase_ != msg->data) {
          const bool failed_drop_return = phase_ == "TASK1_DROP" &&
            msg->data == "TASK1_RETURN_HOME" && !drop_completed_latched_;
          phase_ = msg->data;
          phase_changed_time_ = now();
          if (failed_drop_return) {
            action_failed_latched_ = true;
          }
          if (phase_ == "WAIT_START") {
            drop_completed_latched_ = false;
            c_anchor_accepted_ = false;
            action_failed_latched_ = false;
          }
        }
      });
    active_task_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/competition/active_task_mode", retained,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        if (msg->data <= 2U) {active_task_ = msg->data;}
      });
    flight_sub_ = create_subscription<uav_interfaces::msg::FlightStatus>(
      "/flight/status", qos, [this](const uav_interfaces::msg::FlightStatus::SharedPtr msg) {
        flight_ = *msg;
        flight_time_ = now();
        has_flight_ = true;
      });
    drop_completed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/payload/drop_completed", qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && (phase_ == "TASK1_DROP" || phase_ == "TASK1_RETURN_HOME")) {
          drop_completed_latched_ = true;
        }
      });

    io_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {io_tick();});
    sync_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / sync_rate_)), [this]() {sync_tick();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {status_tick();});

    phase_changed_time_ = now();
    publish_bool(link_pub_, false);
    publish_safe_inputs();
    RCLCPP_INFO(
      get_logger(), "小车串口桥v1.0.2启动：%s，115200 8N1，CAR_STATUS超时%dms",
      device_.c_str(), timeout_ms_);
  }

  ~CarSerialBridgeNode() override {close_port();}

private:
  static void publish_bool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher, bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    publisher->publish(msg);
  }

  bool car_fresh(const rclcpp::Time & current) const
  {
    if (!has_status_) {return false;}
    const double age = (current - status_time_).seconds() * 1000.0;
    return std::isfinite(age) && age >= 0.0 && age <= timeout_ms_;
  }

  bool flight_fresh(const rclcpp::Time & current) const
  {
    if (!has_flight_) {return false;}
    const double age = (current - flight_time_).seconds() * 1000.0;
    return std::isfinite(age) && age >= 0.0 && age <= flight_timeout_ms_ &&
           flight_.communication_ok;
  }

  bool open_port()
  {
    const auto current = now();
    if (last_open_.nanoseconds() != 0 &&
      (current - last_open_).nanoseconds() < static_cast<int64_t>(reconnect_ms_) * 1000000LL)
    {
      return false;
    }
    last_open_ = current;
    const int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "小车串口%s未出现：%s；继续等待",
        device_.c_str(), std::strerror(errno));
      return false;
    }
    termios options{};
    if (tcgetattr(fd, &options) != 0) {::close(fd); return false;}
    cfmakeraw(&options);
    const speed_t speed = baud_constant(baud_);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CRTSCTS | CSIZE));
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &options) != 0) {::close(fd); return false;}
    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    parser_.reset();
    tx_.clear();
    tx_offset_ = 0U;
    RCLCPP_INFO(get_logger(), "小车串口已连接：%s", device_.c_str());
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {::close(fd_);}
    fd_ = -1;
    parser_.reset();
    tx_.clear();
    tx_offset_ = 0U;
    publish_bool(link_pub_, false);
    publish_safe_inputs();
  }

  void flush_tx()
  {
    while (fd_ >= 0 && tx_offset_ < tx_.size()) {
      const ssize_t count = ::write(fd_, tx_.data() + tx_offset_, tx_.size() - tx_offset_);
      if (count > 0) {tx_offset_ += static_cast<std::size_t>(count); continue;}
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(get_logger(), "小车串口写失败：%s", std::strerror(errno));
        close_port();
      }
      return;
    }
    if (tx_offset_ == tx_.size()) {tx_.clear(); tx_offset_ = 0U;}
  }

  void io_tick()
  {
    if (fd_ < 0 && !open_port()) {return;}
    flush_tx();
    std::uint8_t buffer[256];
    while (fd_ >= 0) {
      const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
      if (count > 0) {
        for (const auto & frame : parser_.feed(buffer, static_cast<std::size_t>(count))) {
          handle_frame(frame);
        }
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {break;}
      RCLCPP_ERROR(get_logger(), "小车串口读失败：%s", std::strerror(errno));
      close_port();
      break;
    }
  }

  void handle_frame(const Frame & frame)
  {
    CarStatus decoded;
    if (!decode_car_status(frame, decoded)) {return;}
    if (!valid_session_task(frame.session_id, decoded.task_mode) ||
      !valid_car_status_fields(decoded))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "丢弃非法CAR_STATUS：session=%u task=%u state=0x%02X checkpoint=%u flags=0x%04X",
        frame.session_id, decoded.task_mode, decoded.car_state,
        decoded.last_checkpoint, decoded.flags);
      return;
    }

    latest_ = decoded;
    latest_sequence_ = frame.sequence;
    latest_session_ = frame.session_id;
    status_time_ = now();
    has_status_ = true;
    timed_out_published_ = false;
    publish_bool(link_pub_, true);
    publish_status(true);

    std_msgs::msg::UInt8 task;
    task.data = decoded.task_mode;
    task_pub_->publish(task);

    const bool c_window =
      decoded.last_checkpoint == uav_interfaces::msg::CarStatus::CHECKPOINT_C &&
      (decoded.flags & C_DECEL_COMPLETE) != 0U;
    if (c_window) {c_anchor_accepted_ = true;}
    publish_bool(drop_window_pub_, c_window);
    publish_bool(lap_complete_pub_, (decoded.flags & LAP_COMPLETE) != 0U);

    const bool ready_reset = frame.session_id == 0U && decoded.task_mode == 0U &&
      decoded.mission_phase == kReady && decoded.car_state == 0U;
    if (ready_reset) {
      start_announced_ = false;
      c_anchor_accepted_ = false;
      drop_completed_latched_ = false;
      publish_bool(start_pub_, false);
    }
    if ((decoded.flags & START_LATCHED) != 0U && decoded.task_mode != 0U)
    {
      publish_bool(start_pub_, true);
      if (!start_announced_) {
        start_announced_ = true;
        RCLCPP_INFO(
        get_logger(), "收到小车START：KEY%u/session=%u，启动task%u",
        decoded.task_mode, frame.session_id, decoded.task_mode);
      }
    }

    if (print_status_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "小车RX S=%u task=%u phase=0x%02X state=0x%02X point=%u flags=0x%04X "
        "v=%umm/s d=%umm C窗=%s",
        frame.session_id, decoded.task_mode, decoded.mission_phase, decoded.car_state,
        decoded.last_checkpoint, decoded.flags, decoded.speed_mm_s,
        decoded.distance_since_checkpoint_mm, c_window ? "true" : "false");
    }
  }

  MissionSync make_sync(const rclcpp::Time & current)
  {
    const bool fresh = car_fresh(current);
    MissionSync sync;
    sync.task_mode = active_task_ != 0U ? active_task_ : (fresh ? latest_.task_mode : 0U);
    sync.mission_phase = phase_code(phase_);
    if (phase_ == "WAIT_START" && fresh && latest_.mission_phase == kCountdown) {
      sync.mission_phase = kCountdown;
    }
    const bool f_fresh = flight_fresh(current);
    sync.uav_state = uav_state_for(sync.mission_phase, flight_, f_fresh);
    sync.sync_flags = uav_interfaces::msg::MissionSync::SYNC_VALID;
    const bool active = sync.mission_phase != kReady && sync.mission_phase != kCountdown &&
      sync.mission_phase != kTask1Complete && sync.mission_phase != kTask2Complete &&
      sync.mission_phase != kError;
    if (active) {sync.sync_flags |= uav_interfaces::msg::MissionSync::SYNC_TASK_ACTIVE;}
    if (sync.mission_phase == kTask1Complete || sync.mission_phase == kTask2Complete) {
      sync.sync_flags |= uav_interfaces::msg::MissionSync::SYNC_TASK_COMPLETE;
    }
    if (sync.mission_phase == kError) {
      sync.sync_flags |= uav_interfaces::msg::MissionSync::SYNC_ERROR;
    }
    sync.countdown_remaining_ms =
      sync.mission_phase == kCountdown && fresh ? latest_.countdown_remaining_ms : 0U;

    if (c_anchor_accepted_) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::C_ANCHOR_ACCEPTED;
    }
    if (sync.mission_phase == kTask1AlignDrop || sync.mission_phase == kTask2AlignLand) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::ACTION_PREPARING;
    }
    if (sync.mission_phase == kTask1Drop) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::DROP_IN_PROGRESS;
    }
    if (sync.mission_phase == kTask2Descend) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::LAND_IN_PROGRESS;
    }
    if (sync.mission_phase == kTask2OnCar || sync.mission_phase == kTask2Hold5S ||
      sync.mission_phase == kTask2Complete)
    {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::LANDED_ON_CAR;
    }
    if (sync.mission_phase == kTask2Complete ||
      sync.mission_phase == kTask2Retakeoff || sync.mission_phase == kTask2ReturnH ||
      sync.mission_phase == kTask2LandH)
    {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::HOLD_5S_COMPLETE;
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::CAR_ACCELERATE_ALLOWED;
    }
    if (sync.mission_phase == kTask2Retakeoff) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::RETAKEOFF_IN_PROGRESS;
    }
    if (sync.mission_phase == kTask2ReturnH || sync.mission_phase == kTask2LandH)
    {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::RETAKEOFF_COMPLETED;
    }
    if (drop_completed_latched_) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::DROP_COMPLETED;
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::CAR_ACCELERATE_ALLOWED;
    }
    if (action_failed_latched_) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::ACTION_FAILED;
    }
    if (sync.mission_phase == kTask1LandH || sync.mission_phase == kTask2LandH) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::FINAL_LAND_IN_PROGRESS;
    }
    if (sync.mission_phase == kTask1Complete || sync.mission_phase == kTask2Complete) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::TASK_COMPLETE;
    }
    if (sync.mission_phase == kError) {
      sync.coordination_flags |= uav_interfaces::msg::MissionSync::ACTION_FAILED;
    }

    const double elapsed_ms = (current - phase_changed_time_).seconds() * 1000.0;
    sync.action_elapsed_ms = static_cast<std::uint16_t>(
      std::clamp(elapsed_ms, 0.0, 65535.0));
    sync.error_code = fresh ? 0x00U : 0x02U;
    return sync;
  }

  void sync_tick()
  {
    if (fd_ < 0 || !tx_.empty()) {return;}
    const auto current = now();
    const MissionSync sync = make_sync(current);
    const std::uint16_t session =
      car_fresh(current) ? latest_session_ : static_cast<std::uint16_t>(sync.task_mode);
    const std::uint8_t sequence = tx_sequence_++;
    tx_ = encode_frame(make_mission_sync_frame(sync, sequence, session));
    publish_sync(sync, sequence, session);
    flush_tx();
  }

  void status_tick()
  {
    const bool fresh = car_fresh(now());
    publish_bool(link_pub_, fd_ >= 0 && fresh);
    if (!fresh && !timed_out_published_) {
      publish_status(false);
      publish_safe_inputs();
      timed_out_published_ = true;
      if (has_status_) {
        RCLCPP_WARN(
          get_logger(), "CAR_STATUS超过%dms，已撤销START/C点窗口和旧位置使用", timeout_ms_);
      }
    }
  }

  void publish_status(bool communication_ok)
  {
    uav_interfaces::msg::CarStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = "car";
    msg.communication_ok = communication_ok;
    msg.sequence = latest_sequence_;
    msg.session_id = latest_session_;
    msg.task_mode = latest_.task_mode;
    msg.mission_phase = latest_.mission_phase;
    msg.car_state = latest_.car_state;
    msg.last_checkpoint = latest_.last_checkpoint;
    msg.error_code = latest_.error_code;
    msg.flags = latest_.flags;
    msg.countdown_remaining_ms = latest_.countdown_remaining_ms;
    msg.speed_mm_s = latest_.speed_mm_s;
    msg.action_target_speed_mm_s = latest_.action_target_speed_mm_s;
    msg.distance_since_checkpoint_mm = latest_.distance_since_checkpoint_mm;
    status_pub_->publish(msg);
  }

  void publish_sync(
    const MissionSync & value, std::uint8_t sequence, std::uint16_t session)
  {
    uav_interfaces::msg::MissionSync msg;
    msg.header.stamp = now();
    msg.header.frame_id = "car";
    msg.sequence = sequence;
    msg.session_id = session;
    msg.task_mode = value.task_mode;
    msg.mission_phase = value.mission_phase;
    msg.uav_state = value.uav_state;
    msg.sync_flags = value.sync_flags;
    msg.countdown_remaining_ms = value.countdown_remaining_ms;
    msg.coordination_flags = value.coordination_flags;
    msg.action_elapsed_ms = value.action_elapsed_ms;
    msg.error_code = value.error_code;
    sync_pub_->publish(msg);
  }

  void publish_safe_inputs()
  {
    if (!drop_window_pub_) {return;}
    publish_bool(drop_window_pub_, false);
    publish_bool(lap_complete_pub_, false);
  }

  std::string device_;
  int baud_{115200};
  double sync_rate_{10.0};
  int timeout_ms_{500};
  int reconnect_ms_{1000};
  int flight_timeout_ms_{500};
  bool print_status_{true};
  int fd_{-1};
  StreamParser parser_;
  std::vector<std::uint8_t> tx_;
  std::size_t tx_offset_{0U};
  std::uint8_t tx_sequence_{0U};
  std::uint8_t latest_sequence_{0U};
  std::uint16_t latest_session_{0U};
  std::uint8_t active_task_{0U};
  CarStatus latest_;
  uav_interfaces::msg::FlightStatus flight_;
  std::string phase_{"WAIT_START"};
  bool has_status_{false};
  bool has_flight_{false};
  bool timed_out_published_{false};
  bool start_announced_{false};
  bool c_anchor_accepted_{false};
  bool drop_completed_latched_{false};
  bool action_failed_latched_{false};
  rclcpp::Time status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time flight_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_open_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_changed_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<uav_interfaces::msg::CarStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<uav_interfaces::msg::MissionSync>::SharedPtr sync_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_pub_, drop_window_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lap_complete_pub_, start_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr task_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr active_task_sub_;
  rclcpp::Subscription<uav_interfaces::msg::FlightStatus>::SharedPtr flight_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drop_completed_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_, sync_timer_, status_timer_;
};

}  // namespace car_serial_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<car_serial_bridge::CarSerialBridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("car_serial_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
