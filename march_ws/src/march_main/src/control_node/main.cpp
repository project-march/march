// Copyright 2018 Project March.
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>
#include "ros/ros.h"
#include "main.h"

void gaitInputCallback(const march_custom_msgs::Gait::ConstPtr& msg)
{
  ROS_INFO("I heard: [gait: %ld]", msg->gait);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_node");
  ros::NodeHandle n;

  // @TODO make this an action
  ros::Subscriber sub_gait_input = n.subscribe("master/gait/movement", 1000, gaitInputCallback);
  return 0;
}
