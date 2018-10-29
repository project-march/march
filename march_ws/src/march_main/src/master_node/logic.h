// Copyright 2018 Project March.

#ifndef MARCH_MAIN_LOGIC_H
#define MARCH_MAIN_LOGIC_H

#include <ros/ros.h>
#include "../public/enum/gait_enum.h"

class logic {
  ros::Publisher execute_gait;
 public:
  explicit logic(const ros::Publisher &execute_gait);

  void handel_gait_input(GaitType gaitType);
};

#endif //MARCH_MAIN_LOGIC_H
