#include <cstdint>
#include <vector>
#include <gtest/gtest.h>
#include "car_serial_bridge/protocol.hpp"

using car_serial_bridge::CarStatus;
using car_serial_bridge::Frame;
using car_serial_bridge::MissionSync;
using car_serial_bridge::StreamParser;
using car_serial_bridge::decode_car_status;
using car_serial_bridge::encode_frame;
using car_serial_bridge::make_mission_sync_frame;
using car_serial_bridge::valid_car_status_fields;
using car_serial_bridge::valid_session_task;

TEST(CarProtocol, EncodesFrozenMissionSyncVector)
{
  MissionSync sync;
  sync.task_mode = 1U;
  sync.mission_phase = 0x10U;
  sync.uav_state = 0x04U;
  sync.sync_flags = 0x03U;
  sync.coordination_flags = 0x0003U;
  const auto bytes = encode_frame(make_mission_sync_frame(sync, 0U, 1U));
  const std::vector<std::uint8_t> expected{
    0xDA, 0x7E, 0x01, 0x11, 0x00, 0x0C, 0x00, 0x01,
    0x01, 0x10, 0x04, 0x03, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x3A, 0x0D, 0x0A};
  EXPECT_EQ(bytes, expected);
}

TEST(CarProtocol, DecodesCarStatusAndAcceptsRepeatedSequence)
{
  Frame frame;
  frame.message_type = 0x10U;
  frame.sequence = 7U;
  frame.session_id = 1U;
  frame.payload = {
    1U, 0x07U, 0x04U, 3U, 0U,
    0x00U, 0xDAU, 0x00U, 0x00U,
    0x00U, 0xC8U, 0x00U, 0x96U,
    0x00U, 0x00U, 0x01U, 0x2CU};
  const auto bytes = encode_frame(frame);
  std::vector<std::uint8_t> stream{0xAAU, 0xDAU};
  stream.insert(stream.end(), bytes.begin(), bytes.end());
  stream.insert(stream.end(), bytes.begin(), bytes.end());
  StreamParser parser;
  const auto parsed = parser.feed(stream);
  ASSERT_EQ(parsed.size(), 2U);
  CarStatus status;
  ASSERT_TRUE(decode_car_status(parsed.front(), status));
  EXPECT_TRUE(valid_session_task(parsed.front().session_id, status.task_mode));
  EXPECT_TRUE(valid_car_status_fields(status));
  EXPECT_EQ(status.last_checkpoint, 3U);
  EXPECT_EQ(status.flags, 0x00DAU);
  EXPECT_EQ(status.speed_mm_s, 200U);
  EXPECT_EQ(status.action_target_speed_mm_s, 150U);
  EXPECT_EQ(status.distance_since_checkpoint_mm, 300U);
}

TEST(CarProtocol, RejectsSessionMismatchAndReservedFlags)
{
  EXPECT_FALSE(valid_session_task(1U, 2U));
  CarStatus status;
  status.flags = 0x0800U;
  EXPECT_FALSE(valid_car_status_fields(status));
}

TEST(CarProtocol, BadChecksumResynchronizes)
{
  Frame frame;
  frame.message_type = 0x10U;
  frame.payload.assign(17U, 0U);
  auto bad = encode_frame(frame);
  bad[25] ^= 0x01U;
  const auto good = encode_frame(frame);
  bad.insert(bad.end(), good.begin(), good.end());
  StreamParser parser;
  const auto parsed = parser.feed(bad);
  ASSERT_EQ(parsed.size(), 1U);
}
