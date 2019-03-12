// Copyright 2019 Project March.

#include <march_shared_resources/TopicNames.h>
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "../src/TemperatureSafety.h"

/**
 * Counter
 */
struct ErrorCounter
{
  ErrorCounter() : count(0)
  {
  }

  void cb(const march_shared_resources::Error& msg)
  {
    ROS_INFO("CALLBACK");
    ++count;
  }

  uint32_t count;
};

/**
 * The input for the testcases we want to run.
 */
static const std::vector<std::tuple<float, float, float>> testCases = {
  //      (size, expectedArea, expectedHeight)
  std::make_tuple(1, 1, 1), std::make_tuple(2, 4, 2), std::make_tuple(3, 9, 3)
};

class TestError : public ::testing::Test, public ::testing::WithParamInterface<std::tuple<float, float>>
{
protected:
};

TEST_F(TestError, exceedSpecificThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint1 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint1", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);
  sleep(1);  // wait short period for ros to create the publishers

  sensor_msgs::Temperature msg;
  msg.temperature = 61;
  pub_joint1.publish(msg);

  // Wait to receive message
  sleep(1);
  ros::spinOnce();
  EXPECT_EQ(errorCounter.count, 1);
}

TEST_F(TestError, exceedDefaultThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint1 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint3", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);
  sleep(1);  // wait short period for ros to create the publishers

  sensor_msgs::Temperature msg;
  msg.temperature = 41;
  pub_joint1.publish(msg);

  // Wait to receive message
  sleep(1);
  ros::spinOnce();
  EXPECT_EQ(errorCounter.count, 1);
}

TEST_F(TestError, exceedDefaultThresholdMultipleTimes)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint1 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint3", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);
  sleep(1);  // wait short period for ros to create the publishers

  sensor_msgs::Temperature msg;
  msg.temperature = 41;

  int times = 5;
  for (int i = 0; i < times; i++)
  {
    pub_joint1.publish(msg);
  }

  // Wait to receive message
  sleep(1);
  ros::spinOnce();
  EXPECT_EQ(errorCounter.count, times);
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  ROS_INFO("run main method of test");
  ros::init(argc, argv, "march_safety_test");
  testing::InitGoogleTest(&argc, argv);

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
