// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <march_rqt_gait_generator/MarchGait.h>
#include <march_gait_scheduler/Scheduler.h>

class SchedulerTest : public ::testing::Test {
 protected:
};

/**
 * The main method which runs all the tests
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "testnode");
  testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
