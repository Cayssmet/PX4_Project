#ifndef FLIGHT_CONTROLLER_BRIDGE__PROTOCOL_HPP_
#define FLIGHT_CONTROLLER_BRIDGE__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace flight_controller_bridge
{

// ---------------- V1：保持原16/8字节协议不变 ----------------
constexpr std::size_t kCommandFrameSize = 16U;
constexpr std::size_t kStatusFrameSize = 8U;

constexpr std::uint8_t kCommandHeader0 = 0xAAU;
constexpr std::uint8_t kCommandHeader1 = 0x55U;
constexpr std::uint8_t kStatusHeader0 = 0x55U;
constexpr std::uint8_t kStatusHeader1 = 0xAAU;
constexpr std::uint8_t kTail0 = 0x0DU;
constexpr std::uint8_t kTail1 = 0x0AU;

constexpr std::uint8_t kCommandControlValid = 1U << 0;
constexpr std::uint8_t kCommandLandRequest = 1U << 1;
constexpr std::uint8_t kStatusHeightReached = 1U << 0;
constexpr std::uint8_t kStatusLandAccepted = 1U << 1;
constexpr std::uint8_t kStatusLandCompleted = 1U << 2;
constexpr std::uint8_t kStatusFailsafe = 1U << 3;

struct WireCommand
{
  std::uint8_t sequence{0U};
  bool control_valid{false};
  bool land_request{false};
  std::int16_t velocity_x_mm_s{0};
  std::int16_t velocity_y_mm_s{0};
  std::int16_t yaw_rate_mrad_s{0};
  std::uint16_t target_height_mm{0U};
};

struct WireStatus
{
  std::uint8_t acknowledged_sequence{0U};
  bool height_reached{false};
  bool land_accepted{false};
  bool land_completed{false};
  bool failsafe{false};
};

// ---------------- V2：D-UAV UART V2.0 / revision A ----------------
constexpr std::uint8_t kProtocolVersionV2 = 0x02U;
constexpr std::uint8_t kV2CommandMessageType = 0x01U;
constexpr std::uint8_t kV2StatusMessageType = 0x02U;
constexpr std::size_t kV2CommandFrameSize = 30U;
constexpr std::size_t kV2StatusFrameSize = 30U;
constexpr std::uint8_t kV2CommandHeader0 = 0xA5U;
constexpr std::uint8_t kV2CommandHeader1 = 0x5AU;
constexpr std::uint8_t kV2StatusHeader0 = 0x5AU;
constexpr std::uint8_t kV2StatusHeader1 = 0xA5U;
constexpr std::uint8_t kV2CommandControlValid = 1U << 0;

enum class V2ControlMode : std::uint8_t
{
  kStandby = 0x00U,
  kTakeoff = 0x01U,
  kHeightHold = 0x02U,
  kPrecisionDescent = 0x03U,
  kFinalLand = 0x04U,
  kResetMission = 0x05U,
};

constexpr double kV2TakeoffMinHeightM = 0.3;
constexpr double kV2ContinuousMinHeightM = 0.1;
constexpr double kV2MaxTargetHeightM = 1.7;

constexpr bool v2_target_height_is_valid(V2ControlMode mode, double height_m)
{
  switch (mode) {
    case V2ControlMode::kTakeoff:
      return height_m >= kV2TakeoffMinHeightM && height_m <= kV2MaxTargetHeightM;
    case V2ControlMode::kHeightHold:
    case V2ControlMode::kPrecisionDescent:
      return height_m >= kV2ContinuousMinHeightM && height_m <= kV2MaxTargetHeightM;
    case V2ControlMode::kStandby:
    case V2ControlMode::kFinalLand:
    case V2ControlMode::kResetMission:
      return height_m == 0.0;
  }
  return false;
}

enum class V2ActionResult : std::uint8_t
{
  kNone = 0x00U,
  kPending = 0x01U,
  kSucceeded = 0x02U,
  kRejected = 0x03U,
  kFailed = 0x04U,
};

enum class V2ErrorCode : std::uint8_t
{
  kOk = 0x00U,
  kRcFailsafe = 0x01U,
  kCommandTimeout = 0x02U,
  kHeightTimeout = 0x03U,
  kHeightOutOfRange = 0x04U,
  kCrcOrFrameError = 0x05U,
  kSequenceError = 0x06U,
  kControlNotAuthorized = 0x07U,
  kInvalidMode = 0x08U,
  kInvalidSession = 0x09U,
  kDuplicateAction = 0x0AU,
  kActionNotAllowed = 0x0BU,
  kTakeoffTimeout = 0x0CU,
  kLandTimeout = 0x0DU,
  kResetNotAllowed = 0x0EU,
  kInternalError = 0x0FU,
  kInvalidField = 0x10U,
  kOldAction = 0x11U,
};

constexpr std::uint8_t kV2StatusHeightValid = 1U << 0;
constexpr std::uint8_t kV2StatusHeightReached = 1U << 1;
constexpr std::uint8_t kV2StatusArmed = 1U << 2;
constexpr std::uint8_t kV2StatusAirborne = 1U << 3;
constexpr std::uint8_t kV2StatusLanded = 1U << 4;
constexpr std::uint8_t kV2StatusActionAccepted = 1U << 5;
constexpr std::uint8_t kV2StatusActionCompleted = 1U << 6;
constexpr std::uint8_t kV2StatusFailsafe = 1U << 7;

struct V2WireCommand
{
  std::uint8_t sequence{0U};
  V2ControlMode control_mode{V2ControlMode::kStandby};
  bool control_valid{false};
  std::uint32_t session_id{0U};
  std::uint32_t action_id{0U};
  std::int16_t velocity_x_mm_s{0};
  std::int16_t velocity_y_mm_s{0};
  std::int16_t velocity_z_mm_s{0};
  std::int16_t yaw_rate_mrad_s{0};
  std::uint16_t target_height_mm{0U};
};

struct V2WireStatus
{
  std::uint8_t acknowledged_sequence{0U};
  V2ControlMode current_mode{V2ControlMode::kStandby};
  bool height_valid{false};
  bool height_reached{false};
  bool armed{false};
  bool airborne{false};
  bool landed{false};
  bool action_accepted{false};
  bool action_completed{false};
  bool failsafe{false};
  V2ErrorCode error_code{V2ErrorCode::kOk};
  std::uint32_t active_session_id{0U};
  std::uint32_t active_action_id{0U};
  std::uint16_t current_height_mm{0U};
  std::int16_t vertical_velocity_mm_s{0};
  std::uint16_t height_age_ms{0U};
  std::uint16_t command_age_ms{0U};
  V2ActionResult action_result{V2ActionResult::kNone};
};

enum class SequenceRelation
{
  kDuplicate,
  kNew,
  kOld,
};

std::uint16_t crc16_ccitt_false(const std::uint8_t * data, std::size_t length);

std::array<std::uint8_t, kCommandFrameSize> encode_command_frame(
  const WireCommand & command);
bool decode_status_frame(
  const std::array<std::uint8_t, kStatusFrameSize> & frame,
  WireStatus & status);
std::array<std::uint8_t, kStatusFrameSize> encode_status_frame_for_test(
  const WireStatus & status);

std::array<std::uint8_t, kV2CommandFrameSize> encode_v2_command_frame(
  const V2WireCommand & command);
bool decode_v2_command_frame(
  const std::array<std::uint8_t, kV2CommandFrameSize> & frame,
  V2WireCommand & command);
bool decode_v2_status_frame(
  const std::array<std::uint8_t, kV2StatusFrameSize> & frame,
  V2WireStatus & status);
std::array<std::uint8_t, kV2StatusFrameSize> encode_v2_status_frame_for_test(
  const V2WireStatus & status);

bool acknowledgement_is_recent(
  std::uint8_t last_sent,
  std::uint8_t acknowledged,
  std::uint8_t max_lag_frames);
SequenceRelation classify_transport_sequence(
  std::uint8_t last_sequence,
  std::uint8_t new_sequence);

class StatusStreamParser
{
public:
  std::optional<WireStatus> push(std::uint8_t byte);
  void reset();

private:
  std::vector<std::uint8_t> buffer_;
};

class V2StatusStreamParser
{
public:
  std::optional<V2WireStatus> push(std::uint8_t byte);
  void reset();

private:
  std::vector<std::uint8_t> buffer_;
};

}  // namespace flight_controller_bridge

#endif  // FLIGHT_CONTROLLER_BRIDGE__PROTOCOL_HPP_
