//
// Created by tim on 25-10-18.
//

#include "developer_input_node.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "enum/gait_enum.h"
#include <march_custom_msgs/Gait.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "developer_input_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<march_custom_msgs::Gait>("developer_input", 1000);

  while (ros::ok()) {
    march_custom_msgs::Gait msg;
    msg.gait = GaitType::Sit;
    chatter_pub.publish(msg);
  }

  return 0;
}