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


ros::Time Scheduler::getStartTime(ros::Duration offset)
{
  ros::Time possibleStartingTime = getEndTimeCurrentGait();
  ros::Time currentTime = ros::Time::now();

  if (currentTime > possibleStartingTime)
  {
    if (offset.toNSec() >= 0)
    {
      return currentTime + offset;
    }
    else
    {
      ROS_WARN("Negative offset is ignored, because no gait is currently being executed");
      return currentTime;
    }
  }
  else
  {
    if (possibleStartingTime + offset < currentTime)
    {
      ROS_WARN("Negative offset is partly ignored, otherwise the gait would be scheduled in the past");
      return currentTime;
    }
    else
    {
      return possibleStartingTime + offset;
    }
  }
}
bool Scheduler::lastScheduledGaitInProgress() {
  return ros::Time::now() >= this->startTimeLastGait;
}

control_msgs::FollowJointTrajectoryGoal
Scheduler::scheduleTrajectory(const march_shared_resources::GaitGoal* goal, ros::Duration offset)
{
  ROS_INFO("start time lastgait: %d", this->startTimeLastGait.toSec());
  ROS_INFO("Current time: %d", ros::Time::now().toSec());

  if(!lastScheduledGaitInProgress()){
    throw std::runtime_error("There is already a gait scheduled in the future.");
  }
  ros::Time startingTime = getStartTime(offset);
  trajectory_msgs::JointTrajectory trajectory = setStartTimeGait(goal->current_subgait.trajectory, startingTime);
  control_msgs::FollowJointTrajectoryGoal trajectoryMsg;
  trajectoryMsg.trajectory = trajectory;

  // Update all variables
  this->startTimeLastGait = startingTime;
  this->lastGaitGoal = goal;

  return trajectoryMsg;
}
