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
    ros::Time endTime = this->startLastGait;
    endTime += this->lastGaitGoal->current_subgait.duration;
    return endTime;
  }
  return ros::Time::now();
}

ros::Time Scheduler::getStartTime(ros::Duration offset)
{
  ros::Time endCurrentGait = getEndTimeCurrentGait();
  ros::Time currentTime = ros::Time::now();

  if (currentTime > endCurrentGait)
  {
    if (offset.toNSec() >= 0)
    {
      return currentTime + offset;
    }
    else
    {
      ROS_WARN("Negative offset is ignored, because no gait is currently being "
               "executed");
      return currentTime;
    }
  }
  else
  {
    if (endCurrentGait + offset < currentTime)
    {
      ROS_WARN("Negative offset is partly ignored, otherwise the gait would be "
               "scheduled in the past");
      return currentTime;
    }
    else
    {
      return endCurrentGait + offset;
    }
  }
}
bool Scheduler::lastScheduledGaitNotStarted()
{
  return ros::Time::now() < this->startLastGait;
}

control_msgs::FollowJointTrajectoryGoal Scheduler::scheduleGait(const march_shared_resources::GaitGoal* gaitGoal,
                                                                ros::Duration offset)
{
  ROS_DEBUG("Start time last gait: %f", this->startLastGait.toSec());
  ROS_DEBUG("Current time: %f", ros::Time::now().toSec());

  if (lastScheduledGaitNotStarted())
  {
    throw std::runtime_error("There is already a gait scheduled in the future.");
  }
  ros::Time startTime = getStartTime(offset);
  trajectory_msgs::JointTrajectory trajectory = setStartTimeGait(gaitGoal->current_subgait.trajectory, startTime);
  control_msgs::FollowJointTrajectoryGoal trajectoryMsg;
  trajectoryMsg.trajectory = trajectory;

  this->startLastGait = startTime;
  this->lastGaitGoal = gaitGoal;

  return trajectoryMsg;
}
