// Copyright 2019 Project March.
#include "march_safety/SafetyHandler.h"

SafetyHandler::SafetyHandler(ros::NodeHandle* n, ros::Publisher* errorPublisher, ros::Publisher* soundPublisher)
  : n(n), error_publisher(errorPublisher), sound_publisher(soundPublisher)
{
}

void SafetyHandler::publishFatal(std::string message)
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  error_msg.error_message = message;
  error_msg.type = march_shared_resources::Error::FATAL;

  ROS_ERROR("%s", error_msg.error_message.c_str());
  error_publisher->publish(error_msg);
  march_shared_resources::Sound sound;
  sound.time = ros::Time::now();
  sound.file_name = "fatal.wav";
  sound_publisher->publish(sound);
}

void SafetyHandler::publishNonFatal(std::string message)
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  error_msg.error_message = message;
  error_msg.type = march_shared_resources::Error::NON_FATAL;

  ROS_ERROR("%s", error_msg.error_message.c_str());
  error_publisher->publish(error_msg);
  march_shared_resources::Sound sound;
  sound.time = ros::Time::now();
  sound.file_name = "r2-alarm.wav";
  sound_publisher->publish(sound);
}
