// Copyright 2019 Project March
#ifndef PROJECT_INPUTDEVICESAFETY_H
#define PROJECT_INPUTDEVICESAFETY_H

#include "ros/ros.h"
#include "std_msgs/Time.h"
#include "SafetyType.h"
#include "SafetyHandler.h"
#include <sstream>

#include <march_shared_resources/Error.h>

class InputDeviceSafety : public SafetyType
{
  ros::NodeHandle n;
  SafetyHandler* safety_handler;
  ros::Duration connection_timeout;
  ros::Time time_last_alive;
  ros::Time time_last_send_error;
  ros::Subscriber subscriber_input_device_alive;
  int send_errors_interval;

  void inputDeviceAliveCallback(const std_msgs::TimeConstPtr& msg);

public:
  InputDeviceSafety(ros::NodeHandle* n, SafetyHandler* safety_handler);

  void update();

};

#endif  // PROJECT_INPUTDEVICESAFETY_H
