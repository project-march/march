// Copyright 2018 Project March.

#include "data_logger.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_controller_node");
  ros::NodeHandle n;
  ros::spin();
  return 0;
}
