#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "camera_serial_bridge/protocol.hpp"

namespace camera_serial_bridge
{
namespace
{
struct CameraRequest
{
  std::uint8_t phase{0x00U};
  VisionMode mode{VisionMode::DISABLED};
  std::uint8_t target{0U};
  bool enabled{false};
};

CameraRequest request_for_phase(const std::string & phase)
{
  if (phase == "WAIT_READY") {return {0x02U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TAKEOFF_HOVER") {return {0x03U, VisionMode::DISABLED, 0U, false};}
  if (phase == "RIGHT_SHIFT") {return {0x04U, VisionMode::DISABLED, 0U, false};}
  // 前追阶段提前搜索，发现目标后现有状态机才会切入视觉闭环。
  if (phase == "FORWARD_CHASE") {return {0x05U, VisionMode::SEARCH, 1U, true};}
  if (phase == "TASK1_FOLLOW") {return {0x07U, VisionMode::FOLLOW, 1U, true};}
  if (phase == "TASK1_ALIGN_DROP") {return {0x10U, VisionMode::DROP_ALIGN, 3U, true};}
  if (phase == "TASK1_DROP") {return {0x11U, VisionMode::DROP_ALIGN, 3U, true};}
  if (phase == "TASK1_RETURN_HOME") {return {0x12U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TASK1_LAND_HOME") {return {0x13U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TASK1_COMPLETE") {return {0x14U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TASK2_VISUAL_APPROACH" || phase == "TASK2_ALIGN_LAND" || phase == "TASK2_DESCEND") {return {phase == "TASK2_DESCEND" ? 0x21U : 0x20U, VisionMode::LAND_ALIGN, 2U, true};}
  if (phase == "TASK2_ON_CAR") {return {0x22U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TASK2_HOLD_5S") {return {0x23U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TASK2_RETAKEOFF") {return {0x24U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TASK2_RETURN_H") {return {0x25U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TASK2_LAND_H") {return {0x26U, VisionMode::DISABLED, 0U, false};}
  if (phase == "TASK2_COMPLETE") {return {0x27U, VisionMode::DISABLED, 0U, false};}
  if (phase == "ERROR") {return {0xF1U, VisionMode::DISABLED, 0U, false};}
  return {};
}
}  // namespace

class CameraSerialBridgeNode : public rclcpp::Node
{
public:
  CameraSerialBridgeNode() : Node("camera_serial_bridge")
  {
    device_ = declare_parameter<std::string>("serial_device", "/dev/ttyAMA0");
    baud_ = declare_parameter<int>("baud_rate", 115200);
    command_rate_ = declare_parameter<double>("command_rate_hz", 10.0);
    camera_rate_ = declare_parameter<int>("requested_camera_rate_hz", 20);
    timeout_ms_ = declare_parameter<int>("data_timeout_ms", 200);
    reconnect_ms_ = declare_parameter<int>("reconnect_interval_ms", 1000);
    frame_id_ = declare_parameter<std::string>("body_frame", "base_link");
    print_status_ = declare_parameter<bool>("print_status", true);
    if (device_.empty() || frame_id_.empty() || baud_ != 115200 || command_rate_ <= 0.0 ||
      command_rate_ > 50.0 || camera_rate_ < 1 || camera_rate_ > 100 || timeout_ms_ <= 0 ||
      reconnect_ms_ <= 0)
    {
      throw std::invalid_argument("摄像头串口参数非法；v1.6协议固定使用115200 8N1");
    }

    const auto qos = rclcpp::QoS(10).reliable();
    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/vision/relative_error", qos);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/vision/target_valid", qos);
    drop_pub_ = create_publisher<std_msgs::msg::Bool>("/vision/drop_aligned", qos);
    land_pub_ = create_publisher<std_msgs::msg::Bool>("/vision/land_aligned", qos);
    drop_in_progress_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/payload/drop_in_progress", qos);
    drop_completed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/payload/drop_completed", qos);
    drop_failed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/payload/drop_failed", qos);
    link_pub_ = create_publisher<std_msgs::msg::Bool>("/vision/link_ok", retained);
    state_pub_ = create_publisher<std_msgs::msg::UInt8>("/vision/state", retained);
    target_pub_ = create_publisher<std_msgs::msg::UInt8>("/vision/target_type", retained);
    phase_sub_ = create_subscription<std_msgs::msg::String>(
      "/competition/phase", retained, [this](std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "SAFE_HOLD") {return;}
        const auto next = request_for_phase(msg->data);
        if ((next.mode != request_.mode || next.enabled != request_.enabled) && next.enabled) {
          reset_pending_ = true;
        }
        request_ = next;
      });
    task_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/competition/active_task_mode", retained, [this](std_msgs::msg::UInt8::SharedPtr msg) {
        if (msg->data <= 2U) {
          if (task_ != msg->data && msg->data != 0U) {reset_pending_ = true;}
          task_ = msg->data;
        } else {
          RCLCPP_WARN(get_logger(), "忽略非法task_mode=%u", msg->data);
        }
      });

    io_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {io_tick();});
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / command_rate_)), [this]() {command_tick();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {status_tick();});
    publish_invalid();
    publish_link(false);
    RCLCPP_INFO(get_logger(), "摄像头串口桥v1.0启动：%s，115200 8N1，超时%dms",
      device_.c_str(), timeout_ms_);
  }

  ~CameraSerialBridgeNode() override {close_port();}

private:
  static void publish_bool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher, bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    publisher->publish(msg);
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
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "摄像头串口%s未出现：%s；继续安全等待", device_.c_str(), std::strerror(errno));
      return false;
    }
    termios options{};
    if (tcgetattr(fd, &options) != 0) {::close(fd); return false;}
    cfmakeraw(&options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
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
    RCLCPP_INFO(get_logger(), "摄像头串口已连接：%s", device_.c_str());
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {::close(fd_);}
    fd_ = -1;
    parser_.reset();
    tx_.clear();
    tx_offset_ = 0U;
    publish_invalid();
    publish_link(false);
  }

  void flush_tx()
  {
    while (fd_ >= 0 && tx_offset_ < tx_.size()) {
      const ssize_t count = ::write(fd_, tx_.data() + tx_offset_, tx_.size() - tx_offset_);
      if (count > 0) {tx_offset_ += static_cast<std::size_t>(count); continue;}
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(get_logger(), "摄像头串口写失败：%s", std::strerror(errno));
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
      RCLCPP_ERROR(get_logger(), "摄像头串口读失败：%s", std::strerror(errno));
      close_port();
      break;
    }
  }

  void command_tick()
  {
    if (fd_ < 0 || !tx_.empty()) {return;}
    VisionCommand command;
    command.enable = request_.enabled && (task_ == 1U || task_ == 2U);
    command.mode = command.enable ? request_.mode : VisionMode::DISABLED;
    command.task_mode = task_;
    command.mission_phase = request_.phase;
    command.target_type = command.enable ? request_.target : 0U;
    command.reset_tracker = reset_pending_;
    command.requested_rate_hz = static_cast<std::uint16_t>(camera_rate_);
    tx_ = encode_frame(make_vision_command_frame(command, sequence_++, task_));
    reset_pending_ = false;
    flush_tx();
  }

  void handle_frame(const Frame & frame)
  {
    VisionData data;
    if (!decode_vision_data(frame, data)) {return;}
    if (frame.session_id > 2U || (task_ != 0U && frame.session_id != task_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "丢弃摄像头session：收到%u，当前%u", frame.session_id, task_);
      return;
    }
    last_rx_ = now();
    has_rx_ = true;
    publish_link(true);
    const bool valid = request_.enabled && data.mission_phase_echo == request_.phase &&
      data.target_type == request_.target && (data.flags & 0xC0U) == 0U &&
      (data.flags & TARGET_VALID) && (data.flags & XY_VALID) &&
      (data.flags & IMAGE_FRESH) && !(data.flags & VISION_ERROR);
    geometry_msgs::msg::Vector3Stamped error;
    error.header.stamp = last_rx_;
    error.header.frame_id = frame_id_;
    error.vector.x = valid ? static_cast<double>(data.relative_x_mm) / 1000.0 : 0.0;
    error.vector.y = valid ? static_cast<double>(data.relative_y_mm) / 1000.0 : 0.0;
    error_pub_->publish(error);
    publish_bool(valid_pub_, valid);
    publish_bool(drop_pub_, valid && (data.flags & DROP_ALIGNED));
    publish_bool(land_pub_, valid && (data.flags & LAND_ALIGNED));
    const bool drop_result_matches = request_.enabled && request_.phase == 0x11U &&
      frame.session_id == task_ && data.mission_phase_echo == 0x11U &&
      data.target_type == 0x03U && (data.flags & 0xC0U) == 0U;
    publish_bool(
      drop_in_progress_pub_, drop_result_matches && data.vision_state == 0x07U);
    publish_bool(
      drop_completed_pub_, drop_result_matches && data.vision_state == 0x08U &&
      (data.flags & IMAGE_FRESH) && !(data.flags & VISION_ERROR));
    publish_bool(
      drop_failed_pub_, drop_result_matches &&
      (data.vision_state == 0x09U || data.vision_state == 0xF0U));
    std_msgs::msg::UInt8 state;
    state.data = data.vision_state;
    state_pub_->publish(state);
    std_msgs::msg::UInt8 target;
    target.data = data.target_type;
    target_pub_->publish(target);
    invalid_published_ = !valid;
    if (data.mission_phase_echo != request_.phase) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "摄像头phase回显未同步：camera=0x%02X RPI=0x%02X",
        data.mission_phase_echo, request_.phase);
    }
    if (print_status_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "视觉RX session=%u state=0x%02X flags=0x%02X x=%.3fm y=%.3fm valid=%s",
        frame.session_id, data.vision_state, data.flags, error.vector.x, error.vector.y,
        valid ? "true" : "false");
    }
  }

  void status_tick()
  {
    bool fresh = false;
    if (has_rx_) {
      const double age = (now() - last_rx_).seconds() * 1000.0;
      fresh = std::isfinite(age) && age >= 0.0 && age <= timeout_ms_;
    }
    publish_link(fd_ >= 0 && fresh);
    if (!fresh && !invalid_published_) {
      publish_invalid();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "摄像头数据超时（>%dms），已撤销视觉控制", timeout_ms_);
    }
  }

  void publish_invalid()
  {
    if (!error_pub_) {return;}
    geometry_msgs::msg::Vector3Stamped error;
    error.header.stamp = now();
    error.header.frame_id = frame_id_;
    error_pub_->publish(error);
    publish_bool(valid_pub_, false);
    publish_bool(drop_pub_, false);
    publish_bool(land_pub_, false);
    publish_bool(drop_in_progress_pub_, false);
    publish_bool(drop_completed_pub_, false);
    publish_bool(drop_failed_pub_, false);
    invalid_published_ = true;
  }

  void publish_link(bool value) {link_ok_ = value; publish_bool(link_pub_, value);}

  std::string device_;
  std::string frame_id_;
  int baud_{115200};
  double command_rate_{10.0};
  int camera_rate_{20};
  int timeout_ms_{200};
  int reconnect_ms_{1000};
  bool print_status_{true};
  int fd_{-1};
  StreamParser parser_;
  std::vector<std::uint8_t> tx_;
  std::size_t tx_offset_{0U};
  std::uint8_t sequence_{0U};
  std::uint8_t task_{0U};
  CameraRequest request_;
  bool reset_pending_{false};
  bool has_rx_{false};
  bool link_ok_{false};
  bool invalid_published_{false};
  rclcpp::Time last_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_open_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_, drop_pub_, land_pub_, link_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drop_in_progress_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drop_completed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drop_failed_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_, target_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr task_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_, command_timer_, status_timer_;
};
}  // namespace camera_serial_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<camera_serial_bridge::CameraSerialBridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_serial_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
