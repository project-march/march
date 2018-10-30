// Copyright 2018 Project March.

#include "main.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>

bool gait_instruction(march_custom_msgs::GaitInstruction::Request &request,
                      march_custom_msgs::GaitInstruction::Response &response) {
  ROS_INFO("gait_instruction service called");
  response.result = "Impossible Gait";
  return true;
}

void gaitStatusCallback(const march_custom_msgs::Gait::ConstPtr& msg)
{
  ROS_INFO("I heard: [gait: %ld]", msg->gait);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;

  ros::Publisher input_pub = n.advertise<march_custom_msgs::Gait>("input/gait_input", 1000);
  ros::Publisher play_pub = n.advertise<march_custom_msgs::Gait>("input/play_input", 1000);

  ros::Subscriber sub_gait_status = n.subscribe("gait_status", 1000, gaitStatusCallback);

  ros::ServiceServer gait_input_service = n.advertiseService("input/gait_input", gait_instruction);
  ros::ServiceServer play_input_service = n.advertiseService("input/play_input", gait_instruction);

  ros::spin();
  return 0;
}
