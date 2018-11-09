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
  ros::Publisher left_hip_position_pub = n.advertise<std_msgs::Float64>(TopicNames::left_hip_position, 1000);

  ros::Rate rate(50);

  double count = 0;
  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    std_msgs::Float64 msg;
    count += std::sin(0.1);
    msg.data = count;
    ROS_INFO_ONCE("Publishing");
    left_hip_position_pub.publish(msg);
  }

  return 0;
}
