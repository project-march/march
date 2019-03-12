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
    ++count;
  }

  uint32_t count;
};

class TestError : public ::testing::Test
{
protected:
};

TEST_F(TestError, exceedSpecificThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint1 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint1", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  printf("\n number of pub: %d \n", pub_joint1.getNumSubscribers());
  while (0 == pub_joint1.getNumSubscribers())
  {
    ros::Duration(0.1).sleep();
  }

  sensor_msgs::Temperature msg;
  msg.temperature = 61;
  pub_joint1.publish(msg);

  // Wait to receive message
  ros::Duration duration = ros::Duration(1);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint1", duration);
  ros::spinOnce();

  EXPECT_EQ(1, errorCounter.count);
}

TEST_F(TestError, exceedDefaultThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint3 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint3", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  printf("\n number of pub: %d \n", pub_joint3.getNumSubscribers());
  while (0 == pub_joint3.getNumSubscribers())
  {
    ros::Duration(0.1).sleep();
  }

  sensor_msgs::Temperature msg;
  msg.temperature = 41;
  pub_joint3.publish(msg);


  // Wait to receive message
  ros::Duration duration = ros::Duration(1);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint3", duration);
  ros::spinOnce();

  EXPECT_EQ(1, errorCounter.count);
}

TEST_F(TestError, exceedDefaultThresholdMultipleTimes)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint3 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint3", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  printf("\n number of pub: %d \n", pub_joint3.getNumSubscribers());
  while (0 == pub_joint3.getNumSubscribers())
  {
    ros::Duration(0.1).sleep();
  }

  sensor_msgs::Temperature msg;
  msg.temperature = 41;

  int times = 5;
  for (int i = 0; i < times; i++)
  {
    pub_joint3.publish(msg);
  }

  // Wait to receive message
  ros::Duration duration = ros::Duration(1);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint3", duration);
  ros::spinOnce();

  EXPECT_EQ(times, errorCounter.count);
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  ROS_INFO("run main method of test");
  ros::init(argc, argv, "march_safety_test");
  testing::InitGoogleTest(&argc, argv);
  //WHEN RUNNING ALL TESTS WAITING TIME IS NOT WORKING
  // PUBLISHERS ARE NOT REMOVED??
//  ::testing::GTEST_FLAG(filter) = "TestError.exceedDefaultThresholdMultipleTimes";
  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
