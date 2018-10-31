// Copyright 2018 Project March.

#include <march_custom_msgs/GaitInstruction.h>
#include "gait_controller_node.h"
#include "ros/ros.h"
#include "public/enum/gait_enum.h"

bool gait_instruction(march_custom_msgs::GaitInstruction::Request& request,
                      march_custom_msgs::GaitInstruction::Response& response)
{
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_controller_node");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("gait_instructions", gait_instruction);
  ROS_INFO("gait_controller started! :)");
  ros::spin();
  return 0;
}
