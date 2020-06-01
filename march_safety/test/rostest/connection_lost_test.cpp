// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <iostream>

#include <march_safety/temperature_safety.h>
#include <march_shared_resources/Alive.h>
#include "error_counter.h"

TEST(TestConnectionLost, connectionLost)
{
  ros::Time::init();
  ros::NodeHandle nh;

  double input_device_connection_timeout;
  nh.getParam("/march_safety_node/input_device_connection_timeout", input_device_connection_timeout);
  double send_errors_interval;
  nh.getParam("/march_safety_node/send_errors_interval", send_errors_interval);

  ros::Publisher pub_alive = nh.advertise<march_shared_resources::Alive>("march/input_device/alive", 0);
  ErrorCounter errorCounter;
  ros::Subscriber sub = nh.subscribe("march/error", 0, &ErrorCounter::cb, &errorCounter);

  while (0 == pub_alive.getNumSubscribers() || 0 == sub.getNumPublishers())
  {
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(1u, pub_alive.getNumSubscribers());
  EXPECT_EQ(1u, sub.getNumPublishers());

  march_shared_resources::Alive msg;
  msg.stamp = ros::Time::now();
  msg.id = "test";
  pub_alive.publish(msg);
  ros::spinOnce();
  double sleep_ms = send_errors_interval * 0.9 + input_device_connection_timeout;
  ros::Duration(sleep_ms / 1000).sleep();
  ros::spinOnce();

  EXPECT_EQ(1u, errorCounter.count);
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_safety_test");
  testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
