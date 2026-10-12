#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "gtest/gtest.h"

#include "flight_controller_bridge/protocol.hpp"

namespace fcb = flight_controller_bridge;

TEST(V2HeightLimits, EnforcesModeSpecificProductionBounds)
{
  EXPECT_TRUE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kTakeoff, 0.3));
  EXPECT_TRUE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kTakeoff, 1.7));
  EXPECT_FALSE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kTakeoff, 0.299));
  EXPECT_FALSE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kTakeoff, 1.701));

  EXPECT_TRUE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kHeightHold, 0.1));
  EXPECT_TRUE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kPrecisionDescent, 1.7));
  EXPECT_FALSE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kHeightHold, 0.099));

  EXPECT_TRUE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kStandby, 0.0));
  EXPECT_TRUE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kFinalLand, 0.0));
  EXPECT_TRUE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kResetMission, 0.0));
  EXPECT_FALSE(fcb::v2_target_height_is_valid(fcb::V2ControlMode::kStandby, 0.1));
}

TEST(V2CommandProtocol, EncodesAndDecodesRevisionA)
{
  fcb::V2WireCommand source;
  source.sequence = 0xFEU;
  source.control_mode = fcb::V2ControlMode::kPrecisionDescent;
  source.control_valid = true;
  source.session_id = 0x01020304U;
  source.action_id = 0U;
  source.velocity_x_mm_s = 1000;
  source.velocity_y_mm_s = -500;
  source.velocity_z_mm_s = -120;
  source.yaw_rate_mrad_s = 50;
  source.target_height_mm = 300U;

  const auto frame = fcb::encode_v2_command_frame(source);
  ASSERT_EQ(frame.size(), 30U);
  EXPECT_EQ(frame[0], 0xA5U);
  EXPECT_EQ(frame[1], 0x5AU);
  EXPECT_EQ(frame[2], 0x02U);
  EXPECT_EQ(frame[3], 0x01U);
  EXPECT_EQ(frame[4], 0xFEU);
  EXPECT_EQ(frame[5], 0x03U);
  EXPECT_EQ(frame[6], 0x01U);
  EXPECT_EQ(frame[7], 0x00U);
  EXPECT_EQ(frame[8], 0x01U);
  EXPECT_EQ(frame[9], 0x02U);
  EXPECT_EQ(frame[10], 0x03U);
  EXPECT_EQ(frame[11], 0x04U);
  EXPECT_EQ(frame[16], 0x03U);
  EXPECT_EQ(frame[17], 0xE8U);
  EXPECT_EQ(frame[18], 0xFEU);
  EXPECT_EQ(frame[19], 0x0CU);
  EXPECT_EQ(frame[20], 0xFFU);
  EXPECT_EQ(frame[21], 0x88U);
  EXPECT_EQ(frame[24], 0x01U);
  EXPECT_EQ(frame[25], 0x2CU);
  EXPECT_EQ(frame[28], 0x0DU);
  EXPECT_EQ(frame[29], 0x0AU);
  EXPECT_EQ(
    static_cast<std::uint16_t>((frame[26] << 8) | frame[27]),
    fcb::crc16_ccitt_false(&frame[2], 24U));

  fcb::V2WireCommand decoded;
  ASSERT_TRUE(fcb::decode_v2_command_frame(frame, decoded));
  EXPECT_EQ(decoded.sequence, source.sequence);
  EXPECT_EQ(decoded.control_mode, source.control_mode);
  EXPECT_TRUE(decoded.control_valid);
  EXPECT_EQ(decoded.session_id, source.session_id);
  EXPECT_EQ(decoded.action_id, 0U);
  EXPECT_EQ(decoded.velocity_x_mm_s, 1000);
  EXPECT_EQ(decoded.velocity_y_mm_s, -500);
  EXPECT_EQ(decoded.velocity_z_mm_s, -120);
  EXPECT_EQ(decoded.yaw_rate_mrad_s, 50);
  EXPECT_EQ(decoded.target_height_mm, 300U);
}

TEST(V2StatusProtocol, RoundTripAllFieldsAndRejectsCorruption)
{
  fcb::V2WireStatus source;
  source.acknowledged_sequence = 0x7BU;
  source.current_mode = fcb::V2ControlMode::kHeightHold;
  source.height_valid = true;
  source.height_reached = true;
  source.armed = true;
  source.airborne = true;
  source.landed = false;
  source.action_accepted = true;
  source.action_completed = true;
  source.failsafe = false;
  source.error_code = fcb::V2ErrorCode::kOldAction;
  source.active_session_id = 0x11223344U;
  source.active_action_id = 7U;
  source.current_height_mm = 1500U;
  source.vertical_velocity_mm_s = -123;
  source.height_age_ms = 20U;
  source.command_age_ms = 40U;
  source.action_result = fcb::V2ActionResult::kSucceeded;

  auto frame = fcb::encode_v2_status_frame_for_test(source);
  fcb::V2WireStatus decoded;
  ASSERT_TRUE(fcb::decode_v2_status_frame(frame, decoded));
  EXPECT_EQ(decoded.acknowledged_sequence, 0x7BU);
  EXPECT_EQ(decoded.current_mode, fcb::V2ControlMode::kHeightHold);
  EXPECT_TRUE(decoded.height_valid);
  EXPECT_TRUE(decoded.height_reached);
  EXPECT_TRUE(decoded.armed);
  EXPECT_TRUE(decoded.airborne);
  EXPECT_FALSE(decoded.landed);
  EXPECT_TRUE(decoded.action_accepted);
  EXPECT_TRUE(decoded.action_completed);
  EXPECT_FALSE(decoded.failsafe);
  EXPECT_EQ(decoded.error_code, fcb::V2ErrorCode::kOldAction);
  EXPECT_EQ(decoded.active_session_id, 0x11223344U);
  EXPECT_EQ(decoded.active_action_id, 7U);
  EXPECT_EQ(decoded.current_height_mm, 1500U);
  EXPECT_EQ(decoded.vertical_velocity_mm_s, -123);
  EXPECT_EQ(decoded.height_age_ms, 20U);
  EXPECT_EQ(decoded.command_age_ms, 40U);
  EXPECT_EQ(decoded.action_result, fcb::V2ActionResult::kSucceeded);

  frame[25] = 1U;
  EXPECT_FALSE(fcb::decode_v2_status_frame(frame, decoded));
}

TEST(V2TransportSequence, ClassifiesDuplicateNewOldAndWrap)
{
  EXPECT_EQ(
    fcb::classify_transport_sequence(10U, 10U),
    fcb::SequenceRelation::kDuplicate);
  EXPECT_EQ(
    fcb::classify_transport_sequence(10U, 11U),
    fcb::SequenceRelation::kNew);
  EXPECT_EQ(
    fcb::classify_transport_sequence(255U, 0U),
    fcb::SequenceRelation::kNew);
  EXPECT_EQ(
    fcb::classify_transport_sequence(10U, 9U),
    fcb::SequenceRelation::kOld);
  EXPECT_EQ(
    fcb::classify_transport_sequence(10U, 138U),
    fcb::SequenceRelation::kOld);
}

TEST(V2StatusStreamParser, RecoversFromNoiseAndBadFrame)
{
  fcb::V2WireStatus expected;
  expected.acknowledged_sequence = 42U;
  expected.current_mode = fcb::V2ControlMode::kStandby;
  expected.active_session_id = 1234U;
  const auto good = fcb::encode_v2_status_frame_for_test(expected);
  auto bad = good;
  bad[27] ^= 0x01U;

  std::vector<std::uint8_t> stream{0x00U, 0x5AU, 0x01U, 0x99U};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.push_back(0x5AU);
  stream.insert(stream.end(), good.begin(), good.end());

  fcb::V2StatusStreamParser parser;
  std::optional<fcb::V2WireStatus> result;
  for (const auto byte : stream) {
    if (const auto parsed = parser.push(byte); parsed.has_value()) {
      result = parsed;
    }
  }
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->acknowledged_sequence, 42U);
  EXPECT_EQ(result->active_session_id, 1234U);
}
