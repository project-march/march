//
// Created by tim on 26-10-18.
//

#include "logic.h"
#include "ros/ros.h"

logic::logic(const ros::Publisher &execute_gait) : execute_gait(execute_gait) {}

void logic::handel_gait_input(GaitType gaitType) {
  march_custom_msgs::Gait msg;
  msg.gait = gaitType;
  execute_gait.publish(msg);
}
