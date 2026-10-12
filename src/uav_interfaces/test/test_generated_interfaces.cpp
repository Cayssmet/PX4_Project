#include <gtest/gtest.h>

#include <cstdint>

#include "uav_interfaces/msg/flight_command.hpp"
#include "uav_interfaces/msg/flight_status.hpp"
#include "uav_interfaces/msg/car_status.hpp"
#include "uav_interfaces/msg/gcs_status.hpp"
#include "uav_interfaces/msg/mission_status.hpp"
#include "uav_interfaces/msg/mission_sync.hpp"
#include "uav_interfaces/msg/uav_status.hpp"
#include "uav_interfaces/msg/waypoint.hpp"
#include "uav_interfaces/msg/waypoint_list.hpp"
#include "uav_interfaces/srv/set_waypoint_list.hpp"

TEST(UavInterfaces, WaypointListKeepsExecutionOrder)
{
  uav_interfaces::msg::Waypoint first;
  first.id = 10U;
  first.x = 1.0;
  first.y = 2.0;
  first.yaw = 0.5;
  first.target_height = 1.2;
  first.hold_time = 0.8;

  uav_interfaces::msg::Waypoint second;
  second.id = 20U;
  second.x = 3.0;
  second.y = 4.0;
  second.yaw = -0.5;
  second.target_height = 1.5;
  second.hold_time = 1.0;

  uav_interfaces::msg::WaypointList route;
  route.header.frame_id = "map";
  route.route_id = 7U;
  route.waypoints.push_back(first);
  route.waypoints.push_back(second);

  ASSERT_EQ(route.waypoints.size(), 2U);
  EXPECT_EQ(route.route_id, 7U);
  EXPECT_EQ(route.waypoints[0].id, 10U);
  EXPECT_EQ(route.waypoints[1].id, 20U);
  EXPECT_DOUBLE_EQ(route.waypoints[1].target_height, 1.5);
}

TEST(UavInterfaces, FlightCommandContainsOnlyConfirmedHeightControls)
{
  uav_interfaces::msg::FlightCommand command;
  command.header.frame_id = "base_link";
  command.velocity_x = 0.3F;
  command.velocity_y = -0.1F;
  command.yaw_rate = 0.2F;
  command.target_height = 1.5F;
  command.control_valid = true;
  command.land_request = false;

  EXPECT_EQ(command.header.frame_id, "base_link");
  EXPECT_FLOAT_EQ(command.target_height, 1.5F);
  EXPECT_TRUE(command.control_valid);
  EXPECT_FALSE(command.land_request);
}

TEST(UavInterfaces, MissionStateConstantsAreStable)
{
  using MissionStatus = uav_interfaces::msg::MissionStatus;
  EXPECT_EQ(MissionStatus::WAITING_FOR_ROUTE, static_cast<std::uint8_t>(0U));
  EXPECT_EQ(MissionStatus::WAITING_FOR_PID, static_cast<std::uint8_t>(1U));
  EXPECT_EQ(MissionStatus::TRACKING, static_cast<std::uint8_t>(2U));
  EXPECT_EQ(MissionStatus::DWELLING, static_cast<std::uint8_t>(3U));
  EXPECT_EQ(MissionStatus::COMPLETED, static_cast<std::uint8_t>(4U));
  EXPECT_EQ(MissionStatus::ERROR, static_cast<std::uint8_t>(5U));
}

TEST(UavInterfaces, SetWaypointListServiceReturnsExplicitResult)
{
  uav_interfaces::srv::SetWaypointList::Request request;
  request.waypoint_list.header.frame_id = "map";
  request.waypoint_list.route_id = 8U;

  uav_interfaces::srv::SetWaypointList::Response response;
  response.accepted = true;
  response.message = "已加载航线";

  EXPECT_EQ(request.waypoint_list.route_id, 8U);
  EXPECT_TRUE(response.accepted);
  EXPECT_FALSE(response.message.empty());
}

TEST(UavInterfaces, FlightStatusDoesNotRequireAttitude)
{
  uav_interfaces::msg::FlightStatus status;
  status.communication_ok = true;
  status.height_reached = true;
  status.land_accepted = false;
  status.land_completed = false;
  status.failsafe = false;

  EXPECT_TRUE(status.communication_ok);
  EXPECT_TRUE(status.height_reached);
  EXPECT_FALSE(status.failsafe);
}

TEST(UavInterfaces, CarProtocolConstantsAreStable)
{
  using CarStatus = uav_interfaces::msg::CarStatus;
  using MissionSync = uav_interfaces::msg::MissionSync;
  EXPECT_EQ(CarStatus::CHECKPOINT_C, 3U);
  EXPECT_EQ(CarStatus::FLAG_C_DECEL_COMPLETE, 64U);
  EXPECT_EQ(CarStatus::FLAG_LAP_COMPLETE, 1024U);
  EXPECT_EQ(MissionSync::DROP_COMPLETED, 8U);
  EXPECT_EQ(MissionSync::CAR_ACCELERATE_ALLOWED, 512U);
}

TEST(UavInterfaces, GroundStationProtocolConstantsAreStable)
{
  using UavStatus = uav_interfaces::msg::UavStatus;
  using GcsStatus = uav_interfaces::msg::GcsStatus;
  EXPECT_EQ(UavStatus::PHASE_RIGHT_SHIFT, 4U);
  EXPECT_EQ(UavStatus::PHASE_TASK1_DROP, 17U);
  EXPECT_EQ(UavStatus::UAV_VISUAL_FOLLOW, 4U);
  EXPECT_EQ(UavStatus::FLAG_POSE_VALID, 1U);
  EXPECT_EQ(UavStatus::FLAG_TASK_COMPLETE, 4096U);
  EXPECT_EQ(GcsStatus::GCS_DISPLAYING, 2U);
  EXPECT_EQ(GcsStatus::FLAG_RECORDING, 2U);
}
