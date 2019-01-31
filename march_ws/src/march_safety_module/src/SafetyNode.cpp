// Copyright 2018 Project March.

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Temperature.h"
#include <sstream>

#include <march_shared_resources/TopicNames.h>
#include <march_shared_resources/Error.h>

ros::Publisher error_publisher;
double temperature_threshold;

void temperatureCallback(const sensor_msgs::TemperatureConstPtr& msg, const std::string& sensor_name)
{
  if (msg->temperature > temperature_threshold)
  {
    march_shared_resources::Error error_msg;
    error_msg.error_code = 1;
    std::ostringstream message_stream;
    message_stream << "Temperature of " << sensor_name << " with temperature " << msg->temperature << " degrees is to "
                                                                                                      "high!";
    std::string error_message = message_stream.str();
    ROS_ERROR("%s", error_message.c_str());
    error_msg.error_message = error_message;
    error_publisher.publish(error_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_safety_node");
  ros::NodeHandle n;
  ros::Rate rate(200);

  std::vector<std::string> sensor_names;
  n.getParam("/sensors", sensor_names);
  n.getParam(ros::this_node::getName() + std::string("/temperature_threshold"), temperature_threshold);

  // Create a subscriber for each sensor
  std::vector<ros::Subscriber> temperature_subscribers;

  for (std::string sensor_name : sensor_names)
  {
    // use boost::bind to pass on the sensor_name as extra parameter to the callback method
    ros::Subscriber subscriber_temperature =
        n.subscribe<sensor_msgs::Temperature>(std::string(TopicNames::temperature) + "/" + sensor_name, 1000,
                                              boost::bind(temperatureCallback, _1, sensor_name));
    temperature_subscribers.push_back(subscriber_temperature);
  }

  error_publisher = n.advertise<march_shared_resources::Error>(TopicNames::error, 1000);

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
