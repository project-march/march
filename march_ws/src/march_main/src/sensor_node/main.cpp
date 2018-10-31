// Copyright 2018 Project March.
#include "ros/ros.h"
#include "main.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_node");
  ros::NodeHandle n;
  ros::spin();
  return 0;
}
