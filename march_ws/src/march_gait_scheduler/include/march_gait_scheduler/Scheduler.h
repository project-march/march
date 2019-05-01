// Copyright 2019 Project March.
#ifndef MARCH_GAIT_SCHEDULER_SCHEDULER_H
#define MARCH_GAIT_SCHEDULER_SCHEDULER_H

#include <trajectory_msgs/JointTrajectory.h>
class Scheduler
{

 public:

  static void delayTrajectory(trajectory_msgs::JointTrajectory& trajectory, ros::Duration duration);

};

#endif //MARCH_GAIT_SCHEDULER_SCHEDULER_H
