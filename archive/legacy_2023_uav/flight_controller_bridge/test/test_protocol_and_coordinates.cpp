#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "gtest/gtest.h"

#include "flight_controller_bridge/coordinates.hpp"
#include "flight_controller_bridge/protocol.hpp"

namespace fcb = flight_controller_bridge;

TEST(Crc16, StandardCheckValue)
{
  // CRC16-CCITT-FALSE 对 ASCII "123456789" 的标准校验值应为 0x29B1。
  const std::array<std::uint8_t, 9U> data{
    '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(fcb::crc16_ccitt_false(data.data(), data.size()), 0x29B1U);
}

TEST(CommandProtocol, EncodesFixedLengthBigEndianFrame)
{
  fcb::WireCommand command;
  command.sequence = 0x7AU;
  command.control_valid = true;
  command.land_request = false;
  command.velocity_x_mm_s = 1234;
  command.velocity_y_mm_s = -567;
  command.yaw_rate_mrad_s = -890;
  command.target_height_mm = 1500U;

  const auto frame = fcb::encode_command_frame(command);
  ASSERT_EQ(frame.size(), fcb::kCommandFrameSize);
  EXPECT_EQ(frame[0], 0xAAU);
  EXPECT_EQ(frame[1], 0x55U);
  EXPECT_EQ(frame[2], 0x7AU);
  EXPECT_EQ(frame[3], fcb::kCommandControlValid);
  EXPECT_EQ(frame[4], 0x04U);  // 1234 = 0x04D2
  EXPECT_EQ(frame[5], 0xD2U);
  EXPECT_EQ(frame[6], 0xFDU);  // -567 = 0xFDC9
  EXPECT_EQ(frame[7], 0xC9U);
  EXPECT_EQ(frame[10], 0x05U); // 1500 = 0x05DC
  EXPECT_EQ(frame[11], 0xDCU);
  EXPECT_EQ(frame[14], 0x0DU);
  EXPECT_EQ(frame[15], 0x0AU);

  const std::uint16_t received_crc = static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(frame[12]) << 8) | frame[13]);
  EXPECT_EQ(received_crc, fcb::crc16_ccitt_false(&frame[2], 10U));
}

TEST(StatusProtocol, RoundTripAndRejectsCorruption)
{
  fcb::WireStatus source;
  source.acknowledged_sequence = 201U;
  source.height_reached = true;
  source.land_accepted = true;
  source.land_completed = false;
  source.failsafe = true;

  auto frame = fcb::encode_status_frame_for_test(source);
  fcb::WireStatus decoded;
  ASSERT_TRUE(fcb::decode_status_frame(frame, decoded));
  EXPECT_EQ(decoded.acknowledged_sequence, 201U);
  EXPECT_TRUE(decoded.height_reached);
  EXPECT_TRUE(decoded.land_accepted);
  EXPECT_FALSE(decoded.land_completed);
  EXPECT_TRUE(decoded.failsafe);

  frame[3] ^= fcb::kStatusFailsafe;
  EXPECT_FALSE(fcb::decode_status_frame(frame, decoded));
}

TEST(StatusProtocol, AckWindowHandlesDelayFutureAndSequenceWrap)
{
  EXPECT_TRUE(fcb::acknowledgement_is_recent(20U, 20U, 10U));
  EXPECT_TRUE(fcb::acknowledgement_is_recent(20U, 10U, 10U));
  EXPECT_FALSE(fcb::acknowledgement_is_recent(20U, 9U, 10U));

  // last=2、ack=254 表示跨过 255->0 后落后 4 帧，仍然属于当前会话。
  EXPECT_TRUE(fcb::acknowledgement_is_recent(2U, 254U, 4U));

  // ack=21 位于 last=20 的未来；模减结果为 255，不能误认为近期确认。
  EXPECT_FALSE(fcb::acknowledgement_is_recent(20U, 21U, 10U));
}

TEST(StatusStreamParser, RecoversAfterNoiseAndBadFrame)
{
  fcb::WireStatus expected;
  expected.acknowledged_sequence = 42U;
  expected.height_reached = true;
  const auto valid_frame = fcb::encode_status_frame_for_test(expected);

  auto bad_frame = valid_frame;
  bad_frame[5] ^= 0x01U;

  std::vector<std::uint8_t> stream{0x00U, 0x55U, 0x01U, 0x99U};
  stream.insert(stream.end(), bad_frame.begin(), bad_frame.end());
  stream.push_back(0x55U);  // 额外半个帧头，验证重新同步。
  stream.insert(stream.end(), valid_frame.begin(), valid_frame.end());

  fcb::StatusStreamParser parser;
  std::optional<fcb::WireStatus> result;
  for (const std::uint8_t byte : stream) {
    const auto parsed = parser.push(byte);
    if (parsed.has_value()) {
      result = parsed;
    }
  }

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->acknowledged_sequence, 42U);
  EXPECT_TRUE(result->height_reached);
}

TEST(Coordinates, MapVelocityIsRotatedIntoForwardLeftBodyAxes)
{
  constexpr double kPi = 3.14159265358979323846;

  const auto same = fcb::map_velocity_to_body(1.0, -0.25, 0.0);
  EXPECT_NEAR(same.x, 1.0, 1.0e-12);
  EXPECT_NEAR(same.y, -0.25, 1.0e-12);

  // 机头朝 map +y（yaw=+90°）时，map +x 位于机体右侧，即 body y 为负。
  const auto turned = fcb::map_velocity_to_body(1.0, 0.0, kPi / 2.0);
  EXPECT_NEAR(turned.x, 0.0, 1.0e-12);
  EXPECT_NEAR(turned.y, -1.0, 1.0e-12);

  // 旋转前后水平合速度必须保持一致。
  const auto arbitrary = fcb::map_velocity_to_body(0.6, -0.8, 1.1);
  EXPECT_NEAR(std::hypot(arbitrary.x, arbitrary.y), 1.0, 1.0e-12);
}
