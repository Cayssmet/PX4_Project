#include "flight_controller_bridge/protocol.hpp"

#include <algorithm>

namespace flight_controller_bridge
{
namespace
{

template<std::size_t N>
void write_u16_be(
  std::array<std::uint8_t, N> & frame,
  const std::size_t offset,
  const std::uint16_t value)
{
  frame[offset] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
  frame[offset + 1U] = static_cast<std::uint8_t>(value & 0xFFU);
}

template<std::size_t N>
void write_s16_be(
  std::array<std::uint8_t, N> & frame,
  const std::size_t offset,
  const std::int16_t value)
{
  write_u16_be(frame, offset, static_cast<std::uint16_t>(value));
}

template<std::size_t N>
void write_u32_be(
  std::array<std::uint8_t, N> & frame,
  const std::size_t offset,
  const std::uint32_t value)
{
  frame[offset] = static_cast<std::uint8_t>((value >> 24) & 0xFFU);
  frame[offset + 1U] = static_cast<std::uint8_t>((value >> 16) & 0xFFU);
  frame[offset + 2U] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
  frame[offset + 3U] = static_cast<std::uint8_t>(value & 0xFFU);
}

std::uint16_t read_u16_be(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data[0]) << 8) | data[1]);
}

std::int16_t read_s16_be(const std::uint8_t * data)
{
  return static_cast<std::int16_t>(read_u16_be(data));
}

std::uint32_t read_u32_be(const std::uint8_t * data)
{
  return (static_cast<std::uint32_t>(data[0]) << 24) |
         (static_cast<std::uint32_t>(data[1]) << 16) |
         (static_cast<std::uint32_t>(data[2]) << 8) |
         static_cast<std::uint32_t>(data[3]);
}

bool valid_v2_mode(const std::uint8_t value)
{
  return value <= static_cast<std::uint8_t>(V2ControlMode::kResetMission);
}

bool valid_v2_action_result(const std::uint8_t value)
{
  return value <= static_cast<std::uint8_t>(V2ActionResult::kFailed);
}

}  // namespace

std::uint16_t crc16_ccitt_false(const std::uint8_t * data, const std::size_t length)
{
  std::uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0U; i < length; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<std::uint16_t>((crc << 1) ^ 0x1021U) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::array<std::uint8_t, kCommandFrameSize> encode_command_frame(
  const WireCommand & command)
{
  std::array<std::uint8_t, kCommandFrameSize> frame{};
  frame[0] = kCommandHeader0;
  frame[1] = kCommandHeader1;
  frame[2] = command.sequence;
  frame[3] = static_cast<std::uint8_t>(
    (command.control_valid ? kCommandControlValid : 0U) |
    (command.land_request ? kCommandLandRequest : 0U));
  write_s16_be(frame, 4U, command.velocity_x_mm_s);
  write_s16_be(frame, 6U, command.velocity_y_mm_s);
  write_s16_be(frame, 8U, command.yaw_rate_mrad_s);
  write_u16_be(frame, 10U, command.target_height_mm);
  write_u16_be(frame, 12U, crc16_ccitt_false(&frame[2], 10U));
  frame[14] = kTail0;
  frame[15] = kTail1;
  return frame;
}

bool decode_status_frame(
  const std::array<std::uint8_t, kStatusFrameSize> & frame,
  WireStatus & status)
{
  if (frame[0] != kStatusHeader0 || frame[1] != kStatusHeader1 ||
    frame[6] != kTail0 || frame[7] != kTail1 ||
    read_u16_be(&frame[4]) != crc16_ccitt_false(&frame[2], 2U))
  {
    return false;
  }
  status.acknowledged_sequence = frame[2];
  status.height_reached = (frame[3] & kStatusHeightReached) != 0U;
  status.land_accepted = (frame[3] & kStatusLandAccepted) != 0U;
  status.land_completed = (frame[3] & kStatusLandCompleted) != 0U;
  status.failsafe = (frame[3] & kStatusFailsafe) != 0U;
  return true;
}

std::array<std::uint8_t, kStatusFrameSize> encode_status_frame_for_test(
  const WireStatus & status)
{
  std::array<std::uint8_t, kStatusFrameSize> frame{};
  frame[0] = kStatusHeader0;
  frame[1] = kStatusHeader1;
  frame[2] = status.acknowledged_sequence;
  frame[3] = static_cast<std::uint8_t>(
    (status.height_reached ? kStatusHeightReached : 0U) |
    (status.land_accepted ? kStatusLandAccepted : 0U) |
    (status.land_completed ? kStatusLandCompleted : 0U) |
    (status.failsafe ? kStatusFailsafe : 0U));
  write_u16_be(frame, 4U, crc16_ccitt_false(&frame[2], 2U));
  frame[6] = kTail0;
  frame[7] = kTail1;
  return frame;
}

std::array<std::uint8_t, kV2CommandFrameSize> encode_v2_command_frame(
  const V2WireCommand & command)
{
  std::array<std::uint8_t, kV2CommandFrameSize> frame{};
  frame[0] = kV2CommandHeader0;
  frame[1] = kV2CommandHeader1;
  frame[2] = kProtocolVersionV2;
  frame[3] = kV2CommandMessageType;
  frame[4] = command.sequence;
  frame[5] = static_cast<std::uint8_t>(command.control_mode);
  frame[6] = command.control_valid ? kV2CommandControlValid : 0U;
  frame[7] = 0U;
  write_u32_be(frame, 8U, command.session_id);
  write_u32_be(frame, 12U, command.action_id);
  write_s16_be(frame, 16U, command.velocity_x_mm_s);
  write_s16_be(frame, 18U, command.velocity_y_mm_s);
  write_s16_be(frame, 20U, command.velocity_z_mm_s);
  write_s16_be(frame, 22U, command.yaw_rate_mrad_s);
  write_u16_be(frame, 24U, command.target_height_mm);
  write_u16_be(frame, 26U, crc16_ccitt_false(&frame[2], 24U));
  frame[28] = kTail0;
  frame[29] = kTail1;
  return frame;
}

bool decode_v2_command_frame(
  const std::array<std::uint8_t, kV2CommandFrameSize> & frame,
  V2WireCommand & command)
{
  if (frame[0] != kV2CommandHeader0 || frame[1] != kV2CommandHeader1 ||
    frame[2] != kProtocolVersionV2 || frame[3] != kV2CommandMessageType ||
    frame[7] != 0U || (frame[6] & ~kV2CommandControlValid) != 0U ||
    frame[28] != kTail0 || frame[29] != kTail1 ||
    read_u16_be(&frame[26]) != crc16_ccitt_false(&frame[2], 24U))
  {
    return false;
  }
  command.sequence = frame[4];
  command.control_mode = static_cast<V2ControlMode>(frame[5]);
  command.control_valid = (frame[6] & kV2CommandControlValid) != 0U;
  command.session_id = read_u32_be(&frame[8]);
  command.action_id = read_u32_be(&frame[12]);
  command.velocity_x_mm_s = read_s16_be(&frame[16]);
  command.velocity_y_mm_s = read_s16_be(&frame[18]);
  command.velocity_z_mm_s = read_s16_be(&frame[20]);
  command.yaw_rate_mrad_s = read_s16_be(&frame[22]);
  command.target_height_mm = read_u16_be(&frame[24]);
  return true;
}

bool decode_v2_status_frame(
  const std::array<std::uint8_t, kV2StatusFrameSize> & frame,
  V2WireStatus & status)
{
  if (frame[0] != kV2StatusHeader0 || frame[1] != kV2StatusHeader1 ||
    frame[2] != kProtocolVersionV2 || frame[3] != kV2StatusMessageType ||
    !valid_v2_mode(frame[5]) || !valid_v2_action_result(frame[24]) ||
    frame[25] != 0U || frame[28] != kTail0 || frame[29] != kTail1 ||
    read_u16_be(&frame[26]) != crc16_ccitt_false(&frame[2], 24U))
  {
    return false;
  }
  const std::uint8_t flags = frame[6];
  status.acknowledged_sequence = frame[4];
  status.current_mode = static_cast<V2ControlMode>(frame[5]);
  status.height_valid = (flags & kV2StatusHeightValid) != 0U;
  status.height_reached = (flags & kV2StatusHeightReached) != 0U;
  status.armed = (flags & kV2StatusArmed) != 0U;
  status.airborne = (flags & kV2StatusAirborne) != 0U;
  status.landed = (flags & kV2StatusLanded) != 0U;
  status.action_accepted = (flags & kV2StatusActionAccepted) != 0U;
  status.action_completed = (flags & kV2StatusActionCompleted) != 0U;
  status.failsafe = (flags & kV2StatusFailsafe) != 0U;
  status.error_code = static_cast<V2ErrorCode>(frame[7]);
  status.active_session_id = read_u32_be(&frame[8]);
  status.active_action_id = read_u32_be(&frame[12]);
  status.current_height_mm = read_u16_be(&frame[16]);
  status.vertical_velocity_mm_s = read_s16_be(&frame[18]);
  status.height_age_ms = read_u16_be(&frame[20]);
  status.command_age_ms = read_u16_be(&frame[22]);
  status.action_result = static_cast<V2ActionResult>(frame[24]);
  return true;
}

std::array<std::uint8_t, kV2StatusFrameSize> encode_v2_status_frame_for_test(
  const V2WireStatus & status)
{
  std::array<std::uint8_t, kV2StatusFrameSize> frame{};
  frame[0] = kV2StatusHeader0;
  frame[1] = kV2StatusHeader1;
  frame[2] = kProtocolVersionV2;
  frame[3] = kV2StatusMessageType;
  frame[4] = status.acknowledged_sequence;
  frame[5] = static_cast<std::uint8_t>(status.current_mode);
  frame[6] = static_cast<std::uint8_t>(
    (status.height_valid ? kV2StatusHeightValid : 0U) |
    (status.height_reached ? kV2StatusHeightReached : 0U) |
    (status.armed ? kV2StatusArmed : 0U) |
    (status.airborne ? kV2StatusAirborne : 0U) |
    (status.landed ? kV2StatusLanded : 0U) |
    (status.action_accepted ? kV2StatusActionAccepted : 0U) |
    (status.action_completed ? kV2StatusActionCompleted : 0U) |
    (status.failsafe ? kV2StatusFailsafe : 0U));
  frame[7] = static_cast<std::uint8_t>(status.error_code);
  write_u32_be(frame, 8U, status.active_session_id);
  write_u32_be(frame, 12U, status.active_action_id);
  write_u16_be(frame, 16U, status.current_height_mm);
  write_s16_be(frame, 18U, status.vertical_velocity_mm_s);
  write_u16_be(frame, 20U, status.height_age_ms);
  write_u16_be(frame, 22U, status.command_age_ms);
  frame[24] = static_cast<std::uint8_t>(status.action_result);
  frame[25] = 0U;
  write_u16_be(frame, 26U, crc16_ccitt_false(&frame[2], 24U));
  frame[28] = kTail0;
  frame[29] = kTail1;
  return frame;
}

bool acknowledgement_is_recent(
  const std::uint8_t last_sent,
  const std::uint8_t acknowledged,
  const std::uint8_t max_lag_frames)
{
  return static_cast<std::uint8_t>(last_sent - acknowledged) <= max_lag_frames;
}

SequenceRelation classify_transport_sequence(
  const std::uint8_t last_sequence,
  const std::uint8_t new_sequence)
{
  const auto delta = static_cast<std::uint8_t>(new_sequence - last_sequence);
  if (delta == 0U) {
    return SequenceRelation::kDuplicate;
  }
  return delta <= 127U ? SequenceRelation::kNew : SequenceRelation::kOld;
}

std::optional<WireStatus> StatusStreamParser::push(const std::uint8_t byte)
{
  static constexpr std::array<std::uint8_t, 2U> header{kStatusHeader0, kStatusHeader1};
  buffer_.push_back(byte);
  while (buffer_.size() >= 2U) {
    const auto position = std::search(buffer_.begin(), buffer_.end(), header.begin(), header.end());
    if (position == buffer_.end()) {
      const bool keep_last = buffer_.back() == kStatusHeader0;
      const std::uint8_t last = buffer_.back();
      buffer_.clear();
      if (keep_last) {
        buffer_.push_back(last);
      }
      return std::nullopt;
    }
    buffer_.erase(buffer_.begin(), position);
    if (buffer_.size() < kStatusFrameSize) {
      return std::nullopt;
    }
    std::array<std::uint8_t, kStatusFrameSize> frame{};
    std::copy_n(buffer_.begin(), kStatusFrameSize, frame.begin());
    WireStatus status;
    if (decode_status_frame(frame, status)) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + kStatusFrameSize);
      return status;
    }
    buffer_.erase(buffer_.begin());
  }
  return std::nullopt;
}

void StatusStreamParser::reset()
{
  buffer_.clear();
}

std::optional<V2WireStatus> V2StatusStreamParser::push(const std::uint8_t byte)
{
  static constexpr std::array<std::uint8_t, 2U> header{kV2StatusHeader0, kV2StatusHeader1};
  buffer_.push_back(byte);
  while (buffer_.size() >= 2U) {
    const auto position = std::search(buffer_.begin(), buffer_.end(), header.begin(), header.end());
    if (position == buffer_.end()) {
      const bool keep_last = buffer_.back() == kV2StatusHeader0;
      const std::uint8_t last = buffer_.back();
      buffer_.clear();
      if (keep_last) {
        buffer_.push_back(last);
      }
      return std::nullopt;
    }
    buffer_.erase(buffer_.begin(), position);
    if (buffer_.size() < kV2StatusFrameSize) {
      return std::nullopt;
    }
    std::array<std::uint8_t, kV2StatusFrameSize> frame{};
    std::copy_n(buffer_.begin(), kV2StatusFrameSize, frame.begin());
    V2WireStatus status;
    if (decode_v2_status_frame(frame, status)) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + kV2StatusFrameSize);
      return status;
    }
    buffer_.erase(buffer_.begin());
  }
  return std::nullopt;
}

void V2StatusStreamParser::reset()
{
  buffer_.clear();
}

}  // namespace flight_controller_bridge
