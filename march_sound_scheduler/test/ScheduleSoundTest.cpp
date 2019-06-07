// Copyright 2019 Project March.
#include "ros/ros.h"
#include "gtest/gtest.h"

class ScheduleSoundTest : public ::testing::Test
{
};

TEST_F(ScheduleSoundTest, ScheduleNow)
{
  ros::Time::init();
  ros::Time current_time = ros::Time::now();
//  march_shared_resources::SoundGoal soundGoal;
//  soundGoal.current_subsound.trajectory = fake_sit_trajectory();
//  soundGoal.current_subsound.name = "sit";
//  soundGoal.name = "sit";
//  const auto& soundGoalConst = const_cast<const march_shared_resources::SoundGoal&>(soundGoal);
//
//  Scheduler scheduler;
//  control_msgs::FollowJointTrajectoryGoal trajectoryMsg =
//      scheduler.scheduleSound(&soundGoalConst, ros::Duration().fromSec(0));
//
//  ASSERT_FLOAT_EQ(current_time.toSec(), trajectoryMsg.trajectory.header.stamp.toSec());
}
