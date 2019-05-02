// Copyright 2019 Project March.
#ifndef MARCH_GAIT_SCHEDULER_SCHEDULER_H
#define MARCH_GAIT_SCHEDULER_SCHEDULER_H

#include <ros/duration.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <march_shared_resources/GaitGoal.h>
class Scheduler {

  ros::Time getEndTimeCurrentGait();

  trajectory_msgs::JointTrajectory setStartTimeGait(trajectory_msgs::JointTrajectory trajectory, ros::Time time);

 public:
  const march_shared_resources::GaitGoal *lastGait;
  ros::Time startTimeLastGait = ros::Time::now();
  control_msgs::FollowJointTrajectoryActionGoal scheduleTrajectory(const march_shared_resources::GaitGoal* trajectory);

};

#endif //MARCH_GAIT_SCHEDULER_SCHEDULER_H
