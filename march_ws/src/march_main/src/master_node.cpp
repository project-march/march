//
// Created by tim on 25-10-18.
//

#include "master_node.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "enum/gait_enum.h"
#include <march_custom_msgs/GaitInstruction.h>

bool readyStatus;
GaitType currentGait;

int main(int argc, char **argv) {
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;
  readyStatus = true;
  ros::spin();
  return 0;
}