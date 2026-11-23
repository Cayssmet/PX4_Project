#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "gcs_serial_bridge/protocol.hpp"

using gcs_serial_bridge::Frame;
using gcs_serial_bridge::GcsStatus;
using gcs_serial_bridge::RelativePosition;
using gcs_serial_bridge::StreamParser;
using gcs_serial_bridge::UavStatus;
using gcs_serial_bridge::decode_gcs_status;
using gcs_serial_bridge::encode_frame;
using gcs_serial_bridge::make_uav_status_frame;
using gcs_serial_bridge::relative_position_mm;
using gcs_serial_bridge::valid_gcs_session;
using gcs_serial_bridge::valid_gcs_status_fields;

TEST(GcsProtocol, EncodesFrozenUavStatusVector)
{
  UavStatus status;
  status.task_mode = 1U;
  status.mission_phase = 0x04U;
  status.uav_state = 0x03U;
  status.flags = 0x007BU;
  status.map_x_mm = 375;
  status.map_y_mm = -250;
  status.current_height_mm = 1370U;
  status.mission_elapsed_s = 12U;

  const auto bytes =
    encode_frame(make_uav_status_frame(status, 0xFEU, 1U));
  const std::vector<std::uint8_t> expected{
    0xDA, 0x7E, 0x01, 0x30, 0xFE, 0x12, 0x00, 0x01,
    0x01, 0x04, 0x03, 0x00, 0x00, 0x7B,
    0x00, 0x00, 0x01, 0x77,
    0xFF, 0xFF, 0xFF, 0x06,
    0x05, 0x5A, 0x00, 0x0C, 0xAB, 0x0D, 0x0A};
  EXPECT_EQ(bytes, expected);
}

TEST(GcsProtocol, DecodesHeartbeatAndChecksFields)
{
  Frame heartbeat;
  heartbeat.message_type = 0x31U;
  heartbeat.sequence = 7U;
  heartbeat.session_id = 1U;
  heartbeat.payload = {2U, 0x04U, 0x03U, 0U};

  const auto bytes = encode_frame(heartbeat);
  const std::vector<std::uint8_t> expected{
    0xDA, 0x7E, 0x01, 0x31, 0x07, 0x04, 0x00, 0x01,
    0x02, 0x04, 0x03, 0x00, 0x47, 0x0D, 0x0A};
  EXPECT_EQ(bytes, expected);

  StreamParser parser;
  const auto parsed = parser.feed(bytes);
  ASSERT_EQ(parsed.size(), 1U);
  GcsStatus status;
  ASSERT_TRUE(decode_gcs_status(parsed.front(), status));
  EXPECT_TRUE(valid_gcs_status_fields(status));
  EXPECT_TRUE(valid_gcs_session(parsed.front().session_id, 1U));
  EXPECT_EQ(status.gcs_state, 2U);
  EXPECT_EQ(status.mission_phase_echo, 0x04U);
  EXPECT_EQ(status.flags, 0x03U);
}

TEST(GcsProtocol, PartialBadAndRepeatedFramesResynchronize)
{
  Frame heartbeat;
  heartbeat.message_type = 0x31U;
  heartbeat.sequence = 255U;
  heartbeat.session_id = 2U;
  heartbeat.payload = {1U, 0x20U, 0x01U, 0U};
  const auto good = encode_frame(heartbeat);

  auto bad = good;
  bad[12] ^= 0x01U;
  std::vector<std::uint8_t> prefix{0x00U, 0xDAU, 0x44U};
  prefix.insert(prefix.end(), bad.begin(), bad.end());
  prefix.insert(prefix.end(), good.begin(), good.begin() + 5);

  StreamParser parser;
  EXPECT_TRUE(parser.feed(prefix).empty());

  heartbeat.sequence = 0U;
  const auto wrapped = encode_frame(heartbeat);
  std::vector<std::uint8_t> suffix(good.begin() + 5, good.end());
  suffix.insert(suffix.end(), good.begin(), good.end());
  suffix.insert(suffix.end(), wrapped.begin(), wrapped.end());
  const auto parsed = parser.feed(suffix);
  ASSERT_EQ(parsed.size(), 3U);
  EXPECT_EQ(parsed[0].sequence, 255U);
  EXPECT_EQ(parsed[1].sequence, 255U);
  EXPECT_EQ(parsed[2].sequence, 0U);
}

TEST(GcsProtocol, RejectsReservedHeartbeatFieldsAndSessionMismatch)
{
  GcsStatus status;
  status.gcs_state = 3U;
  EXPECT_FALSE(valid_gcs_status_fields(status));
  status.gcs_state = 1U;
  status.flags = 0x80U;
  EXPECT_FALSE(valid_gcs_status_fields(status));
  EXPECT_FALSE(valid_gcs_session(2U, 1U));
  EXPECT_FALSE(valid_gcs_session(3U, 0U));
}

TEST(GcsProtocol, ConvertsMapPositionIntoInitialBodyAxes)
{
  RelativePosition output;
  constexpr double half_pi = 1.57079632679489661923;
  ASSERT_TRUE(relative_position_mm(
      10.0, 20.0, half_pi, 9.75, 20.375, output));
  EXPECT_EQ(output.x_mm, 375);
  EXPECT_EQ(output.y_mm, 250);
  EXPECT_FALSE(relative_position_mm(
      0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0,
      output));
}

