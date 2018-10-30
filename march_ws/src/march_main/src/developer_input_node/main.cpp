// Copyright 2018 Project March.

#include "main.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "../public/enum/gait_enum.h"
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>


void gaitStatusCallback(const march_custom_msgs::Gait::ConstPtr& msg)
{
  ROS_INFO("I heard: [gait: %ld]", msg->gait);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "developer_input_node");
  ros::NodeHandle n;
  ros::Subscriber sub_gait_status = n.subscribe("gait_status", 1000, gaitStatusCallback);

  ros::ServiceClient gait_input_client = n.serviceClient<march_custom_msgs::GaitInstruction>("input/gait_input");
  ros::ServiceClient play_input_client = n.serviceClient<march_custom_msgs::GaitInstruction>("input/play_input");

  march_custom_msgs::GaitInstruction srv;
  srv.request.gait = 1;
  if (gait_input_client.call(srv))
  {
    const char* output = srv.response.result.c_str();
    ROS_INFO("MasterNode %s", output);
  }
  else
  {
    ROS_ERROR("Failed to call service gait_instructions");
  }

  ROS_INFO("developer_input_node has started");
  return 0;
}
