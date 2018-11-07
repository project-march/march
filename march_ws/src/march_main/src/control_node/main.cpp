// Copyright 2018 Project March.
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>
#include "ros/ros.h"
#include "main.h"
#include "../public/communication/TopicNames.h"
#include <std_msgs/Float64.h>
#include <march_custom_msgs/GaitStatus.h>

void gaitInputCallback(const march_custom_msgs::GaitStatus msg)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_node");
  ros::NodeHandle n;

  ros::Subscriber sub_gait_input = n.subscribe(TopicNames::gait_movement, 1000, gaitInputCallback);
  ros::Publisher gait_input_pub = n.advertise<std_msgs::Float64>(TopicNames::joint_position, 1000);

  ros::Rate rate(1);

  while (ros::ok())
  {
    rate.sleep();
    ros::spin();
    std_msgs::Float64 msg;

    msg.data = 2;
    gait_input_pub.publish(msg);
  }


  return 0;
}
