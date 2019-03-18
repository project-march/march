// Copyright 2019 Project March.

#include <march_shared_resources/TopicNames.h>
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "../src/TemperatureSafety.h"

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

class TestNoError : public ::testing::Test
{
protected:
};

TEST_F(TestNoError, belowSpecificThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint1 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint1", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_joint1.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, pub_joint1.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  msg.temperature = 59;
  pub_joint1.publish(msg);

  // Wait to receive message
  ros::Duration duration = ros::Duration(1);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint1", duration);
  ros::spinOnce();

  EXPECT_EQ(0, errorCounter.count);
}

TEST_F(TestNoError, belowSpecificThreshold2)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint2 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint2", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_joint2.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, pub_joint2.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  msg.temperature = 69;
  pub_joint2.publish(msg);

  // Wait to receive message
  ros::Duration duration = ros::Duration(1);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint2", duration);
  ros::spinOnce();

  EXPECT_EQ(0, errorCounter.count);
}

TEST_F(TestNoError, belowDefaultThreshold)
{
  ros::NodeHandle nh;
  ros::Publisher pub_joint3 = nh.advertise<sensor_msgs::Temperature>("march/temperature/test_joint3", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_joint3.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, pub_joint3.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  sensor_msgs::Temperature msg;
  msg.temperature = 39;
  pub_joint3.publish(msg);

  // Wait to receive message
  ros::Duration duration = ros::Duration(1);
  ros::topic::waitForMessage<sensor_msgs::Temperature>("march/temperature/test_joint3", duration);
  ros::spinOnce();

  EXPECT_EQ(0, errorCounter.count);
}
