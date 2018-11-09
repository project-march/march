// Copyright 2018 Project March.
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>
#include "ros/ros.h"
#include "main.h"
#include "../public/communication/TopicNames.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <march_custom_msgs/GaitStatus.h>
#include <fstream>
#include <iostream>

#include <boost/algorithm/string.hpp>

void gaitInputCallback(const march_custom_msgs::GaitStatus msg)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_node");
  ros::NodeHandle n;

  ros::Subscriber sub_gait_input = n.subscribe(TopicNames::gait_movement, 1000, gaitInputCallback);
  ros::Publisher left_hip_position_pub = n.advertise<std_msgs::Float64>(TopicNames::left_hip_position, 1000);
  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/march/joint_states", 1000);

  ros::Rate rate(100);

  std::string path = "/home/projectmarch/march-iv/march_ws/src/march_main/src/control_node/walking.txt";
  ROS_INFO_STREAM(path);

  std::ifstream file(path);
  if (!file.is_open())
  {
    ROS_ERROR_STREAM("File not open");
  }
  std::string line;

  std::getline(file, line);

  float counter = 0;

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();


    if (std::getline(file, line)) {
      std::vector<std::string> strs;
      boost::split(strs, line, boost::is_any_of("~"));

      sensor_msgs::JointState jointState;
      jointState.position = {stof(strs.at(0)),
                             stof(strs.at(1)),
                             stof(strs.at(2)),
                             stof(strs.at(3)),
                             stof(strs.at(4)),
                             stof(strs.at(5))
      };


      counter += 0.01;
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      jointState.header = header;
//      jointState.name = {"left_hip", "left_knee", "left_ankle", "right_hip", "right_knee", "right_ankle"};
      jointState.name = {"right_hip", "left_hip", "right_knee", "left_knee", "right_ankle", "left_ankle"};
      joint_state_pub.publish(jointState);
    }
  }

  return 0;
}
