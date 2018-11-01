// Copyright 2018 Project March.

#include <march_custom_msgs/GaitInstruction.h>
#include <march_custom_msgs/GaitStatus.h>
#include <march_custom_msgs/GaitInputMaster.h>
#include <march_custom_msgs/PlayInputMaster.h>
#include "main.h"
#include "ros/ros.h"
#include "../public/enum/gait_enum.h"
#include "../public/communication/TopicNames.h"

void gaitInputCallback(const march_custom_msgs::GaitInputMaster::ConstPtr& msg)
{
}

void playInputCallback(const march_custom_msgs::PlayInputMaster::ConstPtr& msg)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_node");
  ros::NodeHandle n;
  ros::Publisher gait_status_pub = n.advertise<march_custom_msgs::GaitStatus>(TopicNames::gait_status, 1000);
  ros::Publisher gait_movement_pub = n.advertise<march_custom_msgs::GaitStatus>(TopicNames::gait_movement, 1000);

  ros::Subscriber sub_gait_input = n.subscribe(TopicNames::gait_input, 1000, gaitInputCallback);
  ros::Subscriber sub_play_input = n.subscribe(TopicNames::play_input, 1000, playInputCallback);

  ros::spin();
  return 0;
}
