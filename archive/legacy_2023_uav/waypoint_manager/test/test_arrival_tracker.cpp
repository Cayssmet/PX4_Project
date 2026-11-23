#include <gtest/gtest.h>

#include "waypoint_manager/arrival_tracker.hpp"

using waypoint_manager::ArrivalTracker;

TEST(ArrivalTracker, RequiresContinuousConfirmationThenHold)
{
  ArrivalTracker tracker(0.7, 0.3);

  EXPECT_FALSE(tracker.update(true, 0.0, 1.0));
  EXPECT_EQ(tracker.stage(), ArrivalTracker::Stage::CONFIRMING);
  EXPECT_FALSE(tracker.update(true, 0.69, 1.0));
  EXPECT_FALSE(tracker.update(true, 0.70, 1.0));
  EXPECT_TRUE(tracker.dwelling());
  EXPECT_FALSE(tracker.update(true, 1.69, 1.0));
  EXPECT_TRUE(tracker.update(true, 1.70, 1.0));
  EXPECT_TRUE(tracker.complete());
}

TEST(ArrivalTracker, ConfirmationIsResetWhenConditionIsLost)
{
  ArrivalTracker tracker(0.7, 0.3);

  EXPECT_FALSE(tracker.update(true, 0.0, 1.0));
  EXPECT_FALSE(tracker.update(false, 0.5, 1.0));
  EXPECT_EQ(tracker.stage(), ArrivalTracker::Stage::SEEKING);
  EXPECT_FALSE(tracker.update(true, 0.6, 1.0));
  EXPECT_FALSE(tracker.update(true, 1.2, 1.0));
  EXPECT_FALSE(tracker.dwelling());
  EXPECT_FALSE(tracker.update(true, 1.3, 1.0));
  EXPECT_TRUE(tracker.dwelling());
}

TEST(ArrivalTracker, BriefLossDuringDwellDoesNotClearProgress)
{
  ArrivalTracker tracker(0.7, 0.3);

  tracker.update(true, 0.0, 1.0);
  tracker.update(true, 0.7, 1.0);
  EXPECT_FALSE(tracker.update(false, 1.0, 1.0));
  EXPECT_FALSE(tracker.update(true, 1.2, 1.0));
  EXPECT_TRUE(tracker.update(true, 1.7, 1.0));
}

TEST(ArrivalTracker, SustainedLossDuringDwellRestartsWholeProcess)
{
  ArrivalTracker tracker(0.7, 0.3);

  tracker.update(true, 0.0, 1.0);
  tracker.update(true, 0.7, 1.0);
  tracker.update(false, 0.8, 1.0);
  EXPECT_FALSE(tracker.update(false, 1.11, 1.0));
  EXPECT_EQ(tracker.stage(), ArrivalTracker::Stage::SEEKING);
  EXPECT_FALSE(tracker.update(true, 1.2, 1.0));
  EXPECT_FALSE(tracker.update(true, 1.89, 1.0));
  EXPECT_FALSE(tracker.dwelling());
}

TEST(ArrivalTracker, TimeGoingBackwardIsSafe)
{
  ArrivalTracker tracker(0.7, 0.3);
  tracker.update(true, 5.0, 1.0);
  EXPECT_FALSE(tracker.update(true, 4.0, 1.0));
  EXPECT_EQ(tracker.stage(), ArrivalTracker::Stage::CONFIRMING);
}
