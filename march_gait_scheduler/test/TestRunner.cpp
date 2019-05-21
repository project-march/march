// Copyright 2018 Project March.

#include "ros/ros.h"
#include "gtest/gtest.h"

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
