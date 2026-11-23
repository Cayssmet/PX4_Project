#include <cstdint>
#include <vector>
#include <gtest/gtest.h>
#include "camera_serial_bridge/protocol.hpp"

using camera_serial_bridge::Frame;
using camera_serial_bridge::StreamParser;
using camera_serial_bridge::VisionCommand;
using camera_serial_bridge::VisionData;
using camera_serial_bridge::VisionMode;
using camera_serial_bridge::decode_vision_data;
using camera_serial_bridge::encode_frame;
using camera_serial_bridge::make_vision_command_frame;

TEST(CameraProtocol, EncodesFixedVisionCommand)
{
  VisionCommand command;
  command.enable = true;
  command.mode = VisionMode::DROP_ALIGN;
  command.task_mode = 1U;
  command.mission_phase = 0x10U;
  command.target_type = 3U;
  command.reset_tracker = true;
  command.requested_rate_hz = 20U;
  const auto bytes = encode_frame(make_vision_command_frame(command, 0x2AU, 1U));
  ASSERT_EQ(bytes.size(), 19U);
  const std::vector<std::uint8_t> expected{
    0xDA, 0x7E, 0x01, 0x20, 0x2A, 0x08, 0x00, 0x01,
    0x01, 0x03, 0x01, 0x10, 0x03, 0x01, 0x00, 0x14,
    0x81, 0x0D, 0x0A};
  EXPECT_EQ(bytes, expected);
}

TEST(CameraProtocol, ParsesSignedVisionDataAndResynchronizes)
{
  Frame frame;
  frame.message_type = 0x21U;
  frame.sequence = 0xFFU;
  frame.session_id = 2U;
  frame.payload = {
    0x20, 0x03, 0x02, 0x1B,
    0x00, 0x00, 0x04, 0xD2,  // +1234 mm
    0xFF, 0xFF, 0xFD, 0xA8}; // -600 mm
  auto bytes = encode_frame(frame);
  std::vector<std::uint8_t> stream{0x00, 0xDA, 0x00, 0x7E};
  stream.insert(stream.end(), bytes.begin(), bytes.end());
  StreamParser parser;
  auto first = parser.feed(stream.data(), 9U);
  EXPECT_TRUE(first.empty());
  auto second = parser.feed(stream.data() + 9U, stream.size() - 9U);
  ASSERT_EQ(second.size(), 1U);
  VisionData data;
  ASSERT_TRUE(decode_vision_data(second.front(), data));
  EXPECT_EQ(second.front().session_id, 2U);
  EXPECT_EQ(data.relative_x_mm, 1234);
  EXPECT_EQ(data.relative_y_mm, -600);
  EXPECT_EQ(data.flags, 0x1BU);
}

TEST(CameraProtocol, RejectsBadChecksumThenFindsNextFrame)
{
  Frame frame;
  frame.message_type = 0x21U;
  frame.payload.assign(12U, 0U);
  auto bad = encode_frame(frame);
  bad[20] ^= 0x01U;
  const auto good = encode_frame(frame);
  bad.insert(bad.end(), good.begin(), good.end());
  StreamParser parser;
  const auto parsed = parser.feed(bad);
  ASSERT_EQ(parsed.size(), 1U);
  EXPECT_EQ(parsed.front().message_type, 0x21U);
}
