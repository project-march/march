// Copyright 2019 Project March
#ifndef PROJECT_INPUTDEVICESAFETY_H
#define PROJECT_INPUTDEVICESAFETY_H

#include "ros/ros.h"
#include "std_msgs/Time.h"
#include <sstream>

#include <march_shared_resources/Error.h>

class InputDeviceSafety
{
  ros::NodeHandle n;
  ros::Publisher* error_publisher;
  ros::Duration connection_timeout;
  ros::Time time_last_alive;
  ros::Time time_last_send_error;
  ros::Subscriber subscriber_input_device_alive;

  void inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg);

  march_shared_resources::Error createErrorMessage();

  void createSubscribers();

public:
  InputDeviceSafety(ros::Publisher* error_publisher, ros::NodeHandle n);

  void checkConnection();
};

#endif  // PROJECT_INPUTDEVICESAFETY_H
