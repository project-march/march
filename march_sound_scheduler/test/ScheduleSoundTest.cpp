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
}