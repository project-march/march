//
// Created by tim on 25-10-18.
//

#include <march_custom_msgs/GaitInstruction.h>
#include "gait_controller_node.h"
#include "ros/ros.h"
#include "enum/gait_enum.h"


GaitType currentGait = Sit;

bool gait_instruction(march_custom_msgs::GaitInstruction::Request &request,
                      march_custom_msgs::GaitInstruction::Response &response) {
  ROS_INFO("gait_instruction service called");
  if(currentGait == Sit){
    GaitType gait = GaitType(request.gait);
    switch(gait){
      case Walk:
        response.result = "Impossible Gait";
        break;
      case Sit:
        response.result = "Already Sitting";
        break;
      case Stand:
        response.result = "Standing";
        break;
    }
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gait_controller_node");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("gait_instructions", gait_instruction);
  ROS_INFO("gait_controller started! :)");
  ros::spin();
  return 0;
}