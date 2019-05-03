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
  if (this->lastGaitGoal != nullptr && this->lastGaitGoal != NULL)
  {
    ros::Time endTime = this->startTimeLastGait;
    endTime += this->lastGaitGoal->current_subgait.duration;
    return endTime;
  }
  return ros::Time().fromSec(0);
}

ros::Time Scheduler::getEarliestStartTime(ros::Time preferred_time)
{
  ros::Time possibleStartingTime = getEndTimeCurrentGait();
  if (preferred_time > getEndTimeCurrentGait())
  {
    return preferred_time;
  }
  else
  {
    return possibleStartingTime;
  }
}

control_msgs::FollowJointTrajectoryActionGoal
Scheduler::scheduleTrajectory(const march_shared_resources::GaitGoal* goal, ros::Time preferred_time)
{
  ros::Time startingTime = getEarliestStartTime(preferred_time);
  trajectory_msgs::JointTrajectory trajectory = setStartTimeGait(goal->current_subgait.trajectory, startingTime);
  control_msgs::FollowJointTrajectoryActionGoal trajectoryMsg;
  trajectoryMsg.goal.trajectory = trajectory;

  // Update all variables
  this->startTimeLastGait = startingTime;
  this->lastGaitGoal = goal;

  return trajectoryMsg;
}
