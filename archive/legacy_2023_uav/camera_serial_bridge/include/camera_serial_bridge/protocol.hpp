#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace camera_serial_bridge
{

constexpr std::uint8_t kProtocolVersion = 0x01U;
constexpr std::uint8_t kVisionCommandType = 0x20U;
constexpr std::uint8_t kVisionDataType = 0x21U;
constexpr std::size_t kVisionCommandPayloadSize = 8U;
constexpr std::size_t kVisionDataPayloadSize = 12U;

enum class VisionMode : std::uint8_t
{
  DISABLED = 0U,
  SEARCH = 1U,
  FOLLOW = 2U,
  DROP_ALIGN = 3U,
  LAND_ALIGN = 4U,
};

enum VisionFlag : std::uint8_t
{
  TARGET_VALID = 1U << 0,
  XY_VALID = 1U << 1,
  DROP_ALIGNED = 1U << 2,
  LAND_ALIGNED = 1U << 3,
  IMAGE_FRESH = 1U << 4,
  VISION_ERROR = 1U << 5,
};

struct Frame
{
  std::uint8_t message_type{0U};
  std::uint8_t sequence{0U};
  std::uint16_t session_id{0U};
  std::vector<std::uint8_t> payload;
};

struct VisionCommand
{
  bool enable{false};
  VisionMode mode{VisionMode::DISABLED};
  std::uint8_t task_mode{0U};
  std::uint8_t mission_phase{0U};
  std::uint8_t target_type{0U};
  bool reset_tracker{false};
  std::uint16_t requested_rate_hz{20U};
};

struct VisionData
{
  std::uint8_t mission_phase_echo{0U};
  std::uint8_t vision_state{0U};
  std::uint8_t target_type{0U};
  std::uint8_t flags{0U};
  std::int32_t relative_x_mm{0};
  std::int32_t relative_y_mm{0};
};

std::vector<std::uint8_t> encode_frame(const Frame & frame);
Frame make_vision_command_frame(
  const VisionCommand & command, std::uint8_t sequence, std::uint16_t session_id);
bool decode_vision_data(const Frame & frame, VisionData & data);

class StreamParser
{
public:
  std::vector<Frame> feed(const std::uint8_t * data, std::size_t size);
  std::vector<Frame> feed(const std::vector<std::uint8_t> & data)
  {
    return feed(data.data(), data.size());
  }
  void reset();

private:
  std::vector<std::uint8_t> buffer_;
};

}  // namespace camera_serial_bridge
