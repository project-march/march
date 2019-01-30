// Copyright 2018 Project March.

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Temperature.h"

#include <march_shared_resources/TopicNames.h>

ros::Publisher error_publisher;

void temperatureCallback(const sensor_msgs::Temperature msg)
{
  if (msg.temperature > 80)
  {

  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "input_device_node");
  ros::NodeHandle n;
  ros::Rate rate(200);

  while (ros::ok())
  {
    rate.sleep();

    ros::Subscriber subscriber_temperature = n.subscribe(TopicNames::temperature, 1000, temperatureCallback);
    ros::Publisher input_device_gait = n.advertise<std_msgs::Empty>(TopicNames::error, 1000);

    ros::spinOnce();
  }

  return 0;
}
