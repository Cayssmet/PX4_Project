#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace car_serial_bridge
{

constexpr std::uint8_t kProtocolVersion = 0x01U;
constexpr std::uint8_t kCarStatusType = 0x10U;
constexpr std::uint8_t kMissionSyncType = 0x11U;
constexpr std::size_t kCarStatusPayloadSize = 17U;
constexpr std::size_t kMissionSyncPayloadSize = 12U;

enum CarFlag : std::uint16_t
{
  START_LATCHED = 1U << 0,
  MOVING = 1U << 1,
  LINE_VALID = 1U << 2,
  DISTANCE_VALID = 1U << 3,
  RPI_SYNC_OK = 1U << 4,
  DECELERATING = 1U << 5,
  C_DECEL_COMPLETE = 1U << 6,
  HOLD_ACTION_SPEED = 1U << 7,
  ACCELERATING = 1U << 8,
  CRUISE_SPEED_REACHED = 1U << 9,
  LAP_COMPLETE = 1U << 10,
  CAR_ERROR = 1U << 12,
  CHECKPOINT_ESTIMATED = 1U << 13,
};

struct Frame
{
  std::uint8_t message_type{0U};
  std::uint8_t sequence{0U};
  std::uint16_t session_id{0U};
  std::vector<std::uint8_t> payload;
};

struct CarStatus
{
  std::uint8_t task_mode{0U};
  std::uint8_t mission_phase{0U};
  std::uint8_t car_state{0U};
  std::uint8_t last_checkpoint{0U};
  std::uint8_t error_code{0U};
  std::uint16_t flags{0U};
  std::uint16_t countdown_remaining_ms{0U};
  std::uint16_t speed_mm_s{0U};
  std::uint16_t action_target_speed_mm_s{0U};
  std::uint32_t distance_since_checkpoint_mm{0U};
};

struct MissionSync
{
  std::uint8_t task_mode{0U};
  std::uint8_t mission_phase{0U};
  std::uint8_t uav_state{0U};
  std::uint8_t sync_flags{0U};
  std::uint16_t countdown_remaining_ms{0U};
  std::uint16_t coordination_flags{0U};
  std::uint16_t action_elapsed_ms{0U};
  std::uint8_t error_code{0U};
};

std::vector<std::uint8_t> encode_frame(const Frame & frame);
Frame make_mission_sync_frame(
  const MissionSync & sync, std::uint8_t sequence, std::uint16_t session_id);
bool decode_car_status(const Frame & frame, CarStatus & status);
bool valid_session_task(std::uint16_t session_id, std::uint8_t task_mode);
bool valid_car_status_fields(const CarStatus & status);

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

}  // namespace car_serial_bridge
