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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "developer_input_node");
  ros::NodeHandle n;
  ros::Subscriber sub_gait_status = n.subscribe(TopicNames::gait_status, 1000, gaitStatusCallback);

  ros::ServiceClient gait_input_client = n.serviceClient<march_custom_msgs::GaitInput>("");
  ros::ServiceClient play_input_client = n.serviceClient<march_custom_msgs::PlayInput>("");

  march_custom_msgs::GaitInput srv;
  srv.request.gait_name = "TODO";
  srv.request.time = 100;
  if (gait_input_client.call(srv))
  {
    const char* output = srv.response.message.c_str();
    ROS_INFO("MasterNode %s", output);
  }
  else
  {
    ROS_ERROR("Failed to call service gait_instructions");
  }

  ROS_INFO("developer_input_node has started");
  return 0;
}
