// Copyright 2019 Project March.
#include "TemperatureSafety.h"

#include <march_shared_resources/TopicNames.h>

TemperatureSafety::TemperatureSafety(ros::Publisher* error_publisher, ros::NodeHandle n)
{
  n.getParam(ros::this_node::getName() + std::string("/temperature_threshold"), temperature_threshold);
  this->error_publisher = error_publisher;
  this->createSubscribers();
}

void TemperatureSafety::temperatureCallback(const sensor_msgs::TemperatureConstPtr& msg, const std::string& sensor_name)
{
  if (msg->temperature > temperature_threshold)
  {
    auto error_msg = createErrorMessage(msg->temperature, sensor_name);
    ROS_ERROR("%i, %s", error_msg.error_code, error_msg.error_message.c_str());
    error_publisher->publish(error_msg);
  }
}

march_shared_resources::Error TemperatureSafety::createErrorMessage(double temperature, const std::string& sensor_name)
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  message_stream << sensor_name << " temperature too high: " << temperature;
  std::string error_message = message_stream.str();
  //@TODO(Tim) Come up with real error codes
  error_msg.error_code = 1;  // For now a randomly chosen error code
  error_msg.error_message = error_message;
  error_msg.type = march_shared_resources::Error::FATAL;
  return error_msg;
}

void TemperatureSafety::createSubscribers()
{
  std::vector<std::string> sensor_names;
  n.getParam("/sensors", sensor_names);
  for (const std::string& sensor_name : sensor_names)
  {
    // Use boost::bind to pass on the sensor_name as extra parameter to the callback method
    ros::Subscriber subscriber_temperature = n.subscribe<sensor_msgs::Temperature>(
        std::string(TopicNames::temperature) + "/" + sensor_name, 1000,
        boost::bind(&TemperatureSafety::temperatureCallback, this, _1, sensor_name));

    temperature_subscribers.push_back(subscriber_temperature);
  }
}
