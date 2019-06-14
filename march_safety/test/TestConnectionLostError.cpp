// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Time.h>
#include <march_shared_resources/TopicNames.h>

#include <march_safety/TemperatureSafety.h>
#include "ErrorCounter.cpp"

class TestConnectionLostError : public ::testing::Test
{
protected:
};

TEST_F(TestConnectionLostError, connectionNotLost)
{
  ros::Time::init();
  ros::NodeHandle nh;
  ros::Publisher pub_alive = nh.advertise<std_msgs::Time>("march/input_device/alive", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_alive.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, pub_alive.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  std_msgs::Time timeMessage;
  timeMessage.data = ros::Time::now();
  pub_alive.publish(timeMessage);
  ros::spinOnce();
  ros::Duration(0.005).sleep();
  ros::spinOnce();
  EXPECT_EQ(0, errorCounter.count);
}

TEST_F(TestConnectionLostError, connectionLost)
{
  ros::Time::init();
  ros::NodeHandle nh;
  ros::Publisher pub_alive = nh.advertise<std_msgs::Time>("march/input_device/alive", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_alive.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1, pub_alive.getNumSubscribers());
  EXPECT_EQ(1, sub.getNumPublishers());

  std_msgs::Time timeMessage;
  timeMessage.data = ros::Time::now();
  pub_alive.publish(timeMessage);
  ros::spinOnce();

  ros::Duration(0.8).sleep();
  ros::spinOnce();

  EXPECT_EQ(1, errorCounter.count);
}
