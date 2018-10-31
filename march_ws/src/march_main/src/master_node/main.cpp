// Copyright 2018 Project March.

#include "main.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "../public/communication/TopicNames.h"
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>
#include <march_custom_msgs/GaitInput.h>
#include <march_custom_msgs/PlayInput.h>

bool gait_instruction(march_custom_msgs::GaitInstruction::Request& request,
                      march_custom_msgs::GaitInstruction::Response& response)
{
  ROS_INFO("gait_instruction service called");
  response.result = "Impossible Gait";
  return true;
}

bool gait_input_service(march_custom_msgs::GaitInput::Request& request,
                        march_custom_msgs::GaitInput::Response& response)
{
  ROS_INFO("gait_instruction service called");
  response.is_successful = static_cast<unsigned char>(true);
  return true;
}

bool play_input_service(march_custom_msgs::PlayInput::Request& request,
                        march_custom_msgs::PlayInput::Response& response)
{
  ROS_INFO("gait_instruction service called");
  response.is_successful = static_cast<unsigned char>(true);
  return true;
}

void gaitStatusCallback(const march_custom_msgs::Gait::ConstPtr& msg)
{
  ROS_INFO("I heard: [gait: %ld]", msg->gait);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;

  ros::Publisher gait_input_pub = n.advertise<march_custom_msgs::Gait>(ServiceNames::gait_input, 1000);
  ros::Publisher play_input_pub = n.advertise<march_custom_msgs::Gait>(ServiceNames::play_input, 1000);

  ros::Subscriber sub_gait_status = n.subscribe(TopicNames::gait_status, 1000, gaitStatusCallback);

  ros::ServiceServer gait_input_service = n.advertiseService(TopicNames::play_input, gait_instruction);
  ros::ServiceServer play_input_service = n.advertiseService(TopicNames::gait_input, gait_instruction);

  ros::spin();
  return 0;
}
