//
// Created by tim on 26-10-18.
//

#ifndef MARCH_MAIN_LOGIC_H
#define MARCH_MAIN_LOGIC_H

#include <ros/ros.h>

class logic {
  ros::Publisher execute_gait;
 public:
  logic(const ros::Publisher &execute_gait);

  void handel_gait_input(GaitType gaitType);
};

#endif //MARCH_MAIN_LOGIC_H
