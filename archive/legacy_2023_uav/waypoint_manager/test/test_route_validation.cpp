#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "waypoint_manager/route_validation.hpp"

namespace
{

uav_interfaces::msg::WaypointList make_valid_route()
{
  constexpr double kPi = 3.14159265358979323846;
  uav_interfaces::msg::WaypointList route;
  route.header.frame_id = "map";
  route.route_id = 10;

  uav_interfaces::msg::Waypoint first;
  first.id = 1;
  first.x = 1.0;
  first.y = 2.0;
  first.yaw = 4.0 * kPi + 0.25;
  first.target_height = 1.5;
  first.hold_time = 1.0;
  route.waypoints.push_back(first);

  uav_interfaces::msg::Waypoint second = first;
  second.id = 2;
  second.x = -2.0;
  route.waypoints.push_back(second);
  return route;
}

}  // namespace

TEST(RouteValidation, AcceptsAndNormalizesCompleteRoute)
{
  waypoint_manager::RouteLimits limits;
  const auto result = waypoint_manager::validate_and_normalize_route(
    make_valid_route(), limits);

  ASSERT_TRUE(result.valid) << result.message;
  ASSERT_EQ(result.normalized_route.waypoints.size(), 2U);
  EXPECT_NEAR(result.normalized_route.waypoints[0].yaw, 0.25, 1.0e-12);
}

TEST(RouteValidation, RejectsWrongFrame)
{
  auto route = make_valid_route();
  route.header.frame_id = "odom";
  waypoint_manager::RouteLimits limits;
  EXPECT_FALSE(waypoint_manager::validate_and_normalize_route(route, limits).valid);
}

TEST(RouteValidation, RejectsDuplicateWaypointId)
{
  auto route = make_valid_route();
  route.waypoints[1].id = route.waypoints[0].id;
  waypoint_manager::RouteLimits limits;
  EXPECT_FALSE(waypoint_manager::validate_and_normalize_route(route, limits).valid);
}

TEST(RouteValidation, RejectsNonFiniteAndOutOfRangeValues)
{
  waypoint_manager::RouteLimits limits;

  auto nonfinite = make_valid_route();
  nonfinite.waypoints[0].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    waypoint_manager::validate_and_normalize_route(nonfinite, limits).valid);

  auto high = make_valid_route();
  high.waypoints[0].target_height = limits.max_target_height + 0.1;
  EXPECT_FALSE(waypoint_manager::validate_and_normalize_route(high, limits).valid);

  auto long_hold = make_valid_route();
  long_hold.waypoints[0].hold_time = limits.max_hold_time + 0.1;
  EXPECT_FALSE(waypoint_manager::validate_and_normalize_route(long_hold, limits).valid);
}

TEST(RouteValidation, RejectsEmptyAndOversizedRoutes)
{
  waypoint_manager::RouteLimits limits;
  uav_interfaces::msg::WaypointList empty;
  empty.header.frame_id = "map";
  EXPECT_FALSE(waypoint_manager::validate_and_normalize_route(empty, limits).valid);

  auto route = make_valid_route();
  limits.max_waypoints = 1U;
  EXPECT_FALSE(waypoint_manager::validate_and_normalize_route(route, limits).valid);
}
