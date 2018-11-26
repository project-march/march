// Copyright 2018 Project March.

#include "main.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "../public/enum/gait_enum.h"
#include "../public/communication/TopicNames.h"
#include <march_custom_msgs/GaitInput.h>
#include <march_custom_msgs/PlayInput.h>
#include <march_custom_msgs/GaitInstruction.h>
#include <march_custom_msgs/GaitStatus.h>

void gaitStatusCallback(const march_custom_msgs::GaitStatus::ConstPtr& msg)
{
  std::string gait_name = msg->gait_name;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "developer_input_node");
  ros::NodeHandle n;

  ROS_INFO("developer_input_node has started");
  ros::spin();
  return 0;
}
