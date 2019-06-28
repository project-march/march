// Copyright 2019 Project March.
#ifndef MARCH_WS_SAFETYHANDLER_H
#define MARCH_WS_SAFETYHANDLER_H

#include <string>
#include <ros/ros.h>
#include <march_shared_resources/Error.h>
#include <march_shared_resources/Sound.h>

class SafetyHandler
{
  ros::NodeHandle* n;
  ros::Publisher* error_publisher;
  ros::Publisher* sound_publisher;

public:
  SafetyHandler(ros::NodeHandle* n, ros::Publisher* errorPublisher, ros::Publisher* soundPublisher);

  void publishFatal(std::string message);

  void publishNonFatal(std::string message);
};

#endif  // MARCH_WS_SAFETYHANDLER_H
