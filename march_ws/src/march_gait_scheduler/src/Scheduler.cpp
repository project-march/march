// Copyright 2019 Project March.
#include "march_gait_scheduler/Scheduler.h"

trajectory_msgs::JointTrajectory Scheduler::setStartTimeGait(trajectory_msgs::JointTrajectory trajectory,
                                                             ros::Time time)
{
  trajectory.header.stamp = time;
  return trajectory;
}

ros::Time Scheduler::getEndTimeCurrentGait()
{
  ros::Time endTime = this->startTimeLastGait;
  if (this->lastGait != nullptr)
  {
    endTime += this->lastGait->current_subgait.duration;
  }
  return endTime;
}

control_msgs::FollowJointTrajectoryActionGoal
Scheduler::scheduleTrajectory(const march_shared_resources::GaitGoal* goal)
{
  this->lastGait = goal;
  ros::Time startingTime = getEndTimeCurrentGait();
  if (ros::Time::now() > getEndTimeCurrentGait())
  {
    startingTime = ros::Time::now();
  }
  this->startTimeLastGait = startingTime;
  trajectory_msgs::JointTrajectory trajectory = setStartTimeGait(goal->current_subgait.trajectory, startingTime);

  control_msgs::FollowJointTrajectoryActionGoal trajectoryMsg;
  trajectoryMsg.goal.trajectory = trajectory;
  return trajectoryMsg;
}
