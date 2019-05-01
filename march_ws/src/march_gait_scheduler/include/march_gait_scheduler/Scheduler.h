// Copyright 2019 Project March.
#ifndef MARCH_GAIT_SCHEDULER_SCHEDULER_H
#define MARCH_GAIT_SCHEDULER_SCHEDULER_H

#include <trajectory_msgs/JointTrajectory.h>

#include <march_rqt_gait_generator/MarchGait.h>

class Scheduler
{

 public:

  static void delayGait(march_rqt_gait_generator::MarchGait& marchGait, ros::Duration delay);

};

#endif //MARCH_GAIT_SCHEDULER_SCHEDULER_H
