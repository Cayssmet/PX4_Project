#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gcs_serial_bridge
{

constexpr std::uint8_t kProtocolVersion = 0x01U;
constexpr std::uint8_t kUavStatusType = 0x30U;
constexpr std::uint8_t kGcsStatusType = 0x31U;
constexpr std::size_t kUavStatusPayloadSize = 18U;
constexpr std::size_t kGcsStatusPayloadSize = 4U;

struct Frame
{
  std::uint8_t message_type{0U};
  std::uint8_t sequence{0U};
  std::uint16_t session_id{0U};
  std::vector<std::uint8_t> payload;
};

struct UavStatus
{
  std::uint8_t task_mode{0U};
  std::uint8_t mission_phase{0U};
  std::uint8_t uav_state{0U};
  std::uint8_t error_code{0U};
  std::uint16_t flags{0U};
  std::int32_t map_x_mm{0};
  std::int32_t map_y_mm{0};
  std::uint16_t current_height_mm{0U};
  std::uint16_t mission_elapsed_s{0U};
};

struct GcsStatus
{
  std::uint8_t gcs_state{0U};
  std::uint8_t mission_phase_echo{0U};
  std::uint8_t flags{0U};
};

struct RelativePosition
{
  std::int32_t x_mm{0};
  std::int32_t y_mm{0};
};

std::vector<std::uint8_t> encode_frame(const Frame & frame);
Frame make_uav_status_frame(
  const UavStatus & status, std::uint8_t sequence, std::uint16_t session_id);
bool decode_gcs_status(const Frame & frame, GcsStatus & status);
bool valid_gcs_session(std::uint16_t received_session, std::uint16_t active_session);
bool valid_gcs_status_fields(const GcsStatus & status);
bool relative_position_mm(
  double home_x_m, double home_y_m, double home_yaw_rad,
  double current_x_m, double current_y_m, RelativePosition & output);

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

}  // namespace gcs_serial_bridge

