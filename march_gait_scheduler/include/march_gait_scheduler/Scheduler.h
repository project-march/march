// Copyright 2019 Project March.
#ifndef MARCH_GAIT_SCHEDULER_SCHEDULER_H
#define MARCH_GAIT_SCHEDULER_SCHEDULER_H

#include <ros/ros.h>
#include <ros/duration.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <march_shared_resources/GaitGoal.h>

class Scheduler {

  const march_shared_resources::GaitGoal *lastGaitGoal = nullptr;
  ros::Time startTimeLastGait;

  ros::Time getEndTimeCurrentGait();
  ros::Time getStartTime(ros::Duration offset);
  bool lastScheduledGaitInProgress();
  static trajectory_msgs::JointTrajectory setStartTimeGait(trajectory_msgs::JointTrajectory trajectory, ros::Time time);

 public:
  control_msgs::FollowJointTrajectoryActionGoal scheduleTrajectory(const march_shared_resources::GaitGoal* trajectory, ros::Duration offset);

};

#endif //MARCH_GAIT_SCHEDULER_SCHEDULER_H
