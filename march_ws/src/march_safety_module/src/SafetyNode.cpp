// Copyright 2018 Project March.

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Temperature.h"
#include <sstream>

#include <march_shared_resources/TopicNames.h>
#include <march_shared_resources/Error.h>

ros::Publisher error_publisher;

void temperatureCallback(const sensor_msgs::Temperature msg)
{
  if (msg.temperature > 80)
  {
    march_shared_resources::Error error_msg;
    error_msg.error_code = 1;
    std::ostringstream message;
    message << "Temperature of " << msg.temperature << " degrees is to high!";
    error_msg.error_message = message.str();
    error_publisher.publish(error_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "input_device_node");
  ros::NodeHandle n;
  ros::Rate rate(200);
  //TopicNames::temperature +
  ros::Subscriber subscriber_temperature = n.subscribe("march/temperature/fake_ankle_joint", 1000, temperatureCallback);
  ros::Publisher input_device_gait = n.advertise<march_shared_resources::Error>(TopicNames::error, 1000);

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
