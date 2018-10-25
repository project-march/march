//
// Created by tim on 25-10-18.
//

#include <march_custom_msgs/GaitInstruction.h>
#include "gait_controller_node.h"
#include "ros/ros.h"
#include "enum/gait_enum.h"


GaitType currentGait;

bool gait_instruction(march_custom_msgs::GaitInstruction::Request &request,
                      march_custom_msgs::GaitInstruction::Response &response) {
  if(currentGait == Sit){
    GaitType gait = GaitType(request.gait);
    switch(gait){
      case Walk:
        response.result = false;
        break;
      case Sit:
        response.result = true;
        break;
      case Stand:
        response.result = true;
        break;
    }
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gait_controller_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("gait_instructions", gait_instruction);

//  ros::Publisher chatter_pub = n.advertise<march_custom_msgs::Gait>("developer_input", 1000);
//
//  while (ros::ok()) {
//    march_custom_msgs::Gait msg;
//    msg.gait = GaitType::Sit;
//    chatter_pub.publish(msg);
//  }
//
  return 0;
}