// Copyright 2019 Project March
#ifndef MARCH_SAFETY_INPUT_DEVICE_SAFETY_H
#define MARCH_SAFETY_INPUT_DEVICE_SAFETY_H
#include "march_safety/safety_type.h"
#include "march_safety/safety_handler.h"

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <march_shared_resources/Alive.h>
#include <march_shared_resources/Error.h>

class InputDeviceSafety : public SafetyType
{
public:
  InputDeviceSafety(ros::NodeHandle* n, SafetyHandler* safety_handler);

  void update(const ros::Time& now) override;

private:
  void inputDeviceAliveCallback(const march_shared_resources::AliveConstPtr& msg);

  SafetyHandler* safety_handler_;

  ros::Subscriber subscriber_input_device_alive_;

  ros::Duration connection_timeout_;

  std::unordered_map<std::string, ros::Time> last_alive_stamps_;
  std::unordered_set<std::string> connected_devices_;
};

#endif  // MARCH_SAFETY_INPUT_DEVICE_SAFETY_H
