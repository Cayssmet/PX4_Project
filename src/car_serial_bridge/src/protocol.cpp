#include "car_serial_bridge/protocol.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>

namespace car_serial_bridge
{
namespace
{
constexpr std::uint8_t kHeader1 = 0xDAU;
constexpr std::uint8_t kHeader2 = 0x7EU;
constexpr std::uint8_t kTail1 = 0x0DU;
constexpr std::uint8_t kTail2 = 0x0AU;
constexpr std::size_t kFrameOverhead = 11U;
constexpr std::size_t kMaximumPayload = 64U;
constexpr std::uint16_t kReservedCarFlags = 0xC800U;

void append_u16_be(std::vector<std::uint8_t> & output, std::uint16_t value)
{
  output.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
  output.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}


std::uint16_t read_u16_be(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data[0]) << 8) | data[1]);
}

std::uint32_t read_u32_be(const std::uint8_t * data)
{
  return (static_cast<std::uint32_t>(data[0]) << 24) |
         (static_cast<std::uint32_t>(data[1]) << 16) |
         (static_cast<std::uint32_t>(data[2]) << 8) |
         static_cast<std::uint32_t>(data[3]);
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
  output.insert(output.end(), {kHeader1, kHeader2, kProtocolVersion, frame.message_type,
    frame.sequence, static_cast<std::uint8_t>(frame.payload.size())});
  append_u16_be(output, frame.session_id);
  output.insert(output.end(), frame.payload.begin(), frame.payload.end());
  output.push_back(checksum8(output, output.size()));
  output.push_back(kTail1);
  output.push_back(kTail2);
  return output;
}

Frame make_mission_sync_frame(
  const MissionSync & sync, std::uint8_t sequence, std::uint16_t session_id)
{
  Frame frame;
  frame.message_type = kMissionSyncType;
  frame.sequence = sequence;
  frame.session_id = session_id;
  frame.payload.reserve(kMissionSyncPayloadSize);
  frame.payload.push_back(sync.task_mode);
  frame.payload.push_back(sync.mission_phase);
  frame.payload.push_back(sync.uav_state);
  frame.payload.push_back(sync.sync_flags);
  append_u16_be(frame.payload, sync.countdown_remaining_ms);
  append_u16_be(frame.payload, sync.coordination_flags);
  append_u16_be(frame.payload, sync.action_elapsed_ms);
  frame.payload.push_back(sync.error_code);
  frame.payload.push_back(0U);
  return frame;
}

bool decode_car_status(const Frame & frame, CarStatus & status)
{
  if (frame.message_type != kCarStatusType || frame.payload.size() != kCarStatusPayloadSize) {
    return false;
  }
  status.task_mode = frame.payload[0];
  status.mission_phase = frame.payload[1];
  status.car_state = frame.payload[2];
  status.last_checkpoint = frame.payload[3];
  status.error_code = frame.payload[4];
  status.flags = read_u16_be(frame.payload.data() + 5U);
  status.countdown_remaining_ms = read_u16_be(frame.payload.data() + 7U);
  status.speed_mm_s = read_u16_be(frame.payload.data() + 9U);
  status.action_target_speed_mm_s = read_u16_be(frame.payload.data() + 11U);
  status.distance_since_checkpoint_mm = read_u32_be(frame.payload.data() + 13U);
  return true;
}

bool valid_session_task(std::uint16_t session_id, std::uint8_t task_mode)
{
  return (session_id == 0U && task_mode == 0U) ||
         (session_id == 1U && task_mode == 1U) ||
         (session_id == 2U && task_mode == 2U);
}

bool valid_car_status_fields(const CarStatus & status)
{
  const bool valid_car_state = status.car_state <= 6U || status.car_state == 0xF0U;
  return status.task_mode <= 2U && status.last_checkpoint <= 4U &&
         valid_car_state && status.countdown_remaining_ms <= 3000U &&
         (status.flags & kReservedCarFlags) == 0U;
}

std::vector<Frame> StreamParser::feed(const std::uint8_t * data, std::size_t size)
{
  if (data != nullptr && size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }
  std::vector<Frame> frames;
  constexpr std::array<std::uint8_t, 2> header{kHeader1, kHeader2};
  while (true) {
    const auto position = std::search(buffer_.begin(), buffer_.end(), header.begin(), header.end());
    if (position == buffer_.end()) {
      if (!buffer_.empty() && buffer_.back() == kHeader1) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        buffer_.clear();
      }
      break;
    }
    buffer_.erase(buffer_.begin(), position);
    if (buffer_.size() < 8U) {break;}
    const std::size_t payload_size = buffer_[5];
    if (buffer_[2] != kProtocolVersion || payload_size > kMaximumPayload) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t total_size = kFrameOverhead + payload_size;
    if (buffer_.size() < total_size) {break;}
    const std::size_t checksum_index = 8U + payload_size;
    if (buffer_[checksum_index] != checksum8(buffer_, checksum_index) ||
      buffer_[total_size - 2U] != kTail1 || buffer_[total_size - 1U] != kTail2)
    {
      buffer_.erase(buffer_.begin());
      continue;
    }
    Frame frame;
    frame.message_type = buffer_[3];
    frame.sequence = buffer_[4];
    frame.session_id = read_u16_be(buffer_.data() + 6U);
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

}  // namespace car_serial_bridge
