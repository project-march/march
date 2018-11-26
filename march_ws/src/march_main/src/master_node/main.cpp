// Copyright 2018 Project March.

#include "main.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "../public/communication/TopicNames.h"
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>
#include <march_custom_msgs/GaitInput.h>
#include <march_custom_msgs/PlayInput.h>
#include <march_custom_msgs/PlotDemo.h>
#include <march_custom_msgs/GaitStatus.h>
#include <march_custom_msgs/GaitInputMaster.h>
#include <march_custom_msgs/PlayInputMaster.h>

bool gait_input_callback(march_custom_msgs::GaitInput::Request& request,
                         march_custom_msgs::GaitInput::Response& response)
{
  ROS_INFO("gait input service call received");
  response.is_successful = static_cast<unsigned char>(true);
  return true;
}

bool play_input_callback(march_custom_msgs::PlayInput::Request& request,
                         march_custom_msgs::PlayInput::Response& response)
{
  ROS_INFO("gait_instruction service called");
  response.is_successful = static_cast<unsigned char>(true);
  return true;
}

void gaitStatusCallback(const march_custom_msgs::GaitStatus::ConstPtr& msg)
{
}

int main(int argc, char** argv)
{
  ROS_INFO("Started");
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;

  return 0;
}
