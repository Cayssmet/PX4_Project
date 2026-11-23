#include "gcs_serial_bridge/protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gcs_serial_bridge
{
namespace
{
constexpr std::uint8_t kHeader1 = 0xDAU;
constexpr std::uint8_t kHeader2 = 0x7EU;
constexpr std::uint8_t kTail1 = 0x0DU;
constexpr std::uint8_t kTail2 = 0x0AU;
constexpr std::size_t kFrameOverhead = 11U;
constexpr std::size_t kMaximumPayload = 64U;

void append_u16_be(std::vector<std::uint8_t> & output, std::uint16_t value)
{
  output.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

void append_i32_be(std::vector<std::uint8_t> & output, std::int32_t value)
{
  const auto raw = static_cast<std::uint32_t>(value);
  output.push_back(static_cast<std::uint8_t>((raw >> 24) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>((raw >> 16) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>(raw & 0xFFU));
}

std::uint16_t read_u16_be(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data[0]) << 8) | data[1]);
}

std::uint8_t checksum8(const std::vector<std::uint8_t> & bytes, std::size_t end)
{
  std::uint8_t checksum = 0U;
  for (std::size_t index = 2U; index < end; ++index) {
    checksum = static_cast<std::uint8_t>(checksum + bytes[index]);
  }
  return checksum;
}

bool millimetres_fit_i32(double metres)
{
  constexpr double minimum =
    static_cast<double>(std::numeric_limits<std::int32_t>::min()) / 1000.0;
  constexpr double maximum =
    static_cast<double>(std::numeric_limits<std::int32_t>::max()) / 1000.0;
  return std::isfinite(metres) && metres >= minimum && metres <= maximum;
}

}  // namespace

std::vector<std::uint8_t> encode_frame(const Frame & frame)
{
  if (frame.payload.size() > std::numeric_limits<std::uint8_t>::max()) {
    throw std::invalid_argument("payload过长");
  }

  std::vector<std::uint8_t> output;
  output.reserve(kFrameOverhead + frame.payload.size());
  output.insert(
    output.end(),
    {kHeader1, kHeader2, kProtocolVersion, frame.message_type, frame.sequence,
      static_cast<std::uint8_t>(frame.payload.size())});
  append_u16_be(output, frame.session_id);
  output.insert(output.end(), frame.payload.begin(), frame.payload.end());
  output.push_back(checksum8(output, output.size()));
  output.push_back(kTail1);
  output.push_back(kTail2);
  return output;
}

Frame make_uav_status_frame(
  const UavStatus & status, std::uint8_t sequence, std::uint16_t session_id)
{
  Frame frame;
  frame.message_type = kUavStatusType;
  frame.sequence = sequence;
  frame.session_id = session_id;
  frame.payload.reserve(kUavStatusPayloadSize);
  frame.payload.push_back(status.task_mode);
  frame.payload.push_back(status.mission_phase);
  frame.payload.push_back(status.uav_state);
  frame.payload.push_back(status.error_code);
  append_u16_be(frame.payload, status.flags);
  append_i32_be(frame.payload, status.map_x_mm);
  append_i32_be(frame.payload, status.map_y_mm);
  append_u16_be(frame.payload, status.current_height_mm);
  append_u16_be(frame.payload, status.mission_elapsed_s);
  return frame;
}

bool decode_gcs_status(const Frame & frame, GcsStatus & status)
{
  if (frame.message_type != kGcsStatusType ||
    frame.payload.size() != kGcsStatusPayloadSize)
  {
    return false;
  }
  status.gcs_state = frame.payload[0];
  status.mission_phase_echo = frame.payload[1];
  status.flags = frame.payload[2];
  return frame.payload[3] == 0U;
}

bool valid_gcs_session(std::uint16_t received_session, std::uint16_t active_session)
{
  if (received_session > 2U || active_session > 2U) {
    return false;
  }
  return received_session == active_session;
}

bool valid_gcs_status_fields(const GcsStatus & status)
{
  const bool state_valid =
    status.gcs_state <= 2U || status.gcs_state == 0xF0U;
  return state_valid && (status.flags & 0xFCU) == 0U;
}

bool relative_position_mm(
  double home_x_m, double home_y_m, double home_yaw_rad,
  double current_x_m, double current_y_m, RelativePosition & output)
{
  if (!std::isfinite(home_x_m) || !std::isfinite(home_y_m) ||
    !std::isfinite(home_yaw_rad) || !std::isfinite(current_x_m) ||
    !std::isfinite(current_y_m))
  {
    return false;
  }

  const double delta_x = current_x_m - home_x_m;
  const double delta_y = current_y_m - home_y_m;
  const double cosine = std::cos(home_yaw_rad);
  const double sine = std::sin(home_yaw_rad);
  const double forward_m = cosine * delta_x + sine * delta_y;
  const double left_m = -sine * delta_x + cosine * delta_y;
  if (!millimetres_fit_i32(forward_m) || !millimetres_fit_i32(left_m)) {
    return false;
  }

  output.x_mm = static_cast<std::int32_t>(std::llround(forward_m * 1000.0));
  output.y_mm = static_cast<std::int32_t>(std::llround(left_m * 1000.0));
  return true;
}

std::vector<Frame> StreamParser::feed(const std::uint8_t * data, std::size_t size)
{
  if (data != nullptr && size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<Frame> frames;
  constexpr std::array<std::uint8_t, 2> header{kHeader1, kHeader2};
  while (true) {
    const auto position =
      std::search(buffer_.begin(), buffer_.end(), header.begin(), header.end());
    if (position == buffer_.end()) {
      if (!buffer_.empty() && buffer_.back() == kHeader1) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        buffer_.clear();
      }
      break;
    }

    buffer_.erase(buffer_.begin(), position);
    if (buffer_.size() < 8U) {
      break;
    }

    const std::size_t payload_size = buffer_[5];
    if (buffer_[2] != kProtocolVersion || payload_size > kMaximumPayload) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t total_size = kFrameOverhead + payload_size;
    if (buffer_.size() < total_size) {
      break;
    }

    const std::size_t checksum_index = 8U + payload_size;
    if (buffer_[checksum_index] != checksum8(buffer_, checksum_index) ||
      buffer_[total_size - 2U] != kTail1 ||
      buffer_[total_size - 1U] != kTail2)
    {
      buffer_.erase(buffer_.begin());
      continue;
    }

    Frame frame;
    frame.message_type = buffer_[3];
    frame.sequence = buffer_[4];
    frame.session_id = read_u16_be(buffer_.data() + 6U);
    frame.payload.assign(
      buffer_.begin() + 8, buffer_.begin() + checksum_index);
    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_size);
  }

  return frames;
}

void StreamParser::reset()
{
  buffer_.clear();
}

}  // namespace gcs_serial_bridge

