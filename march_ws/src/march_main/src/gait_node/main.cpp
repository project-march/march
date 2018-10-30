// Copyright 2018 Project March.

#include <march_custom_msgs/GaitInstruction.h>
#include <march_custom_msgs/Gait.h>
#include "main.h"
#include "ros/ros.h"
#include "../public/enum/gait_enum.h"

void gaitInputCallback(const march_custom_msgs::Gait::ConstPtr& msg)
{
  ROS_INFO("I heard: [gait: %ld]", msg->gait);
}

void playInputCallback(const march_custom_msgs::Gait::ConstPtr& msg)
{
  ROS_INFO("I heard: [gait: %ld]", msg->gait);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_controller_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<march_custom_msgs::Gait>("gait/status", 1000);

  ros::Subscriber sub_gait_input = n.subscribe("master/gait_input", 1000, gaitInputCallback);
  ros::Subscriber sub_play_input = n.subscribe("master/play_input", 1000, playInputCallback);

  // @TODO make this an action
  ros::Publisher input_pub = n.advertise<march_custom_msgs::Gait>("gait/movement", 1000);

  return 0;
}
