#include "camera_serial_bridge/protocol.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>

namespace camera_serial_bridge
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

std::int32_t read_i32_be(const std::uint8_t * data)
{
  const std::uint32_t value =
    (static_cast<std::uint32_t>(data[0]) << 24) |
    (static_cast<std::uint32_t>(data[1]) << 16) |
    (static_cast<std::uint32_t>(data[2]) << 8) |
    static_cast<std::uint32_t>(data[3]);
  return static_cast<std::int32_t>(value);
}

std::uint8_t checksum8(const std::vector<std::uint8_t> & bytes, std::size_t end)
{
  std::uint8_t checksum = 0U;
  for (std::size_t i = 2U; i < end; ++i) {
    checksum = static_cast<std::uint8_t>(checksum + bytes[i]);
  }
  return checksum;
}

}  // namespace

std::vector<std::uint8_t> encode_frame(const Frame & frame)
{
  if (frame.payload.size() > std::numeric_limits<std::uint8_t>::max()) {
    throw std::invalid_argument("payload过长");
  }

  std::vector<std::uint8_t> output;
  output.reserve(kFrameOverhead + frame.payload.size());
  output.push_back(kHeader1);
  output.push_back(kHeader2);
  output.push_back(kProtocolVersion);
  output.push_back(frame.message_type);
  output.push_back(frame.sequence);
  output.push_back(static_cast<std::uint8_t>(frame.payload.size()));
  append_u16_be(output, frame.session_id);
  output.insert(output.end(), frame.payload.begin(), frame.payload.end());
  output.push_back(checksum8(output, output.size()));
  output.push_back(kTail1);
  output.push_back(kTail2);
  return output;
}

Frame make_vision_command_frame(
  const VisionCommand & command, std::uint8_t sequence, std::uint16_t session_id)
{
  Frame frame;
  frame.message_type = kVisionCommandType;
  frame.sequence = sequence;
  frame.session_id = session_id;
  frame.payload.reserve(kVisionCommandPayloadSize);
  frame.payload.push_back(command.enable ? 1U : 0U);
  frame.payload.push_back(static_cast<std::uint8_t>(command.mode));
  frame.payload.push_back(command.task_mode);
  frame.payload.push_back(command.mission_phase);
  frame.payload.push_back(command.target_type);
  frame.payload.push_back(command.reset_tracker ? 1U : 0U);
  append_u16_be(frame.payload, command.requested_rate_hz);
  return frame;
}

bool decode_vision_data(const Frame & frame, VisionData & data)
{
  if (frame.message_type != kVisionDataType ||
    frame.payload.size() != kVisionDataPayloadSize)
  {
    return false;
  }
  data.mission_phase_echo = frame.payload[0];
  data.vision_state = frame.payload[1];
  data.target_type = frame.payload[2];
  data.flags = frame.payload[3];
  data.relative_x_mm = read_i32_be(frame.payload.data() + 4U);
  data.relative_y_mm = read_i32_be(frame.payload.data() + 8U);
  return true;
}

std::vector<Frame> StreamParser::feed(const std::uint8_t * data, std::size_t size)
{
  if (data != nullptr && size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<Frame> frames;
  constexpr std::array<std::uint8_t, 2> header_bytes{kHeader1, kHeader2};
  while (true) {
    const auto header = std::search(
      buffer_.begin(), buffer_.end(),
      header_bytes.begin(), header_bytes.end());
    if (header == buffer_.end()) {
      if (!buffer_.empty() && buffer_.back() == kHeader1) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        buffer_.clear();
      }
      break;
    }
    buffer_.erase(buffer_.begin(), header);
    if (buffer_.size() < 8U) {
      break;
    }

    const std::size_t payload_size = buffer_[5];
    if (payload_size > kMaximumPayload || buffer_[2] != kProtocolVersion) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t total_size = kFrameOverhead + payload_size;
    if (buffer_.size() < total_size) {
      break;
    }
    const std::size_t checksum_index = 8U + payload_size;
    if (buffer_[total_size - 2U] != kTail1 || buffer_[total_size - 1U] != kTail2 ||
      buffer_[checksum_index] != checksum8(buffer_, checksum_index))
    {
      buffer_.erase(buffer_.begin());
      continue;
    }

    Frame frame;
    frame.message_type = buffer_[3];
    frame.sequence = buffer_[4];
    frame.session_id = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(buffer_[6]) << 8) | buffer_[7]);
    frame.payload.assign(buffer_.begin() + 8, buffer_.begin() + checksum_index);
    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_size);
  }
  return frames;
}

void StreamParser::reset()
{
  buffer_.clear();
}

}  // namespace camera_serial_bridge
