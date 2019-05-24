// Copyright 2018 Project March.

#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float64.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/algorithm/string.hpp>
#include <trajectory_msgs/JointTrajectory.h>

#include <march_gait_scheduler/Scheduler.h>
#include <march_shared_resources/GaitAction.h>
#include <march_shared_resources/GaitGoal.h>
#include <march_shared_resources/TopicNames.h>

typedef actionlib::SimpleActionServer<march_shared_resources::GaitAction> ScheduleGaitActionServer;
ScheduleGaitActionServer* schedule_gait_action_server;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* followJointTrajectoryAction;
Scheduler* scheduler;

void doneCallback(const actionlib::SimpleClientGoalState& state,
                  const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_DEBUG("Gait trajectory execution DONE");
  if (schedule_gait_action_server->isActive())
  {
    schedule_gait_action_server->setSucceeded();
    ROS_WARN("Schedule gait action SUCCEEDED");
  }
}

void activeCallback()
{
  ROS_DEBUG("Gait trajectory goal just went active");
}

void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  if (scheduler->getEndTimeCurrentGait().toSec() - feedback->header.stamp.toSec() <
      scheduler->GAIT_SUCCEEDED_OFFSET.toSec())
  {
    if (schedule_gait_action_server->isActive())
    {
      schedule_gait_action_server->setSucceeded();
      ROS_DEBUG("Schedule gait action SUCCEEDED");
    }
  }
}

void scheduleGaitCallback(const march_shared_resources::GaitGoalConstPtr& goal)
{
  ROS_DEBUG("Gait Scheduler received msg to schedule: %s of %s", goal->current_subgait.name.c_str(),
            goal->name.c_str());
  try
  {
    control_msgs::FollowJointTrajectoryGoal trajectoryMsg =
        scheduler->scheduleTrajectory(goal.get(), ros::Duration().fromSec(0));
    followJointTrajectoryAction->sendGoal(trajectoryMsg, &doneCallback, &activeCallback, &feedbackCallback);
    ROS_DEBUG("follow joint trajectory action called");
  }
  catch (std::runtime_error error)
  {
    ROS_ERROR("When trying to schedule a trajectory the following error occurred: %s", error.what());
    ROS_ERROR("Gait scheduling ABORTED");
    schedule_gait_action_server->setAborted();
    return;
  }

  while (schedule_gait_action_server->isActive())
  {
    ros::Rate r(100);
    r.sleep();
    ROS_DEBUG_THROTTLE(1, "Waiting on gait execution");
  }
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_scheduler_node");
  ros::NodeHandle n;
  ros::Rate rate(100);

  scheduler = new Scheduler();

  followJointTrajectoryAction = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
      "/march/trajectory_controller/follow_joint_trajectory", true);

  ROS_INFO("Wait on joint trajectory action server");
  bool isConnected = followJointTrajectoryAction->waitForServer(ros::Duration(10));
  if (isConnected)
  {
    ROS_INFO("Connected to joint trajectory action server");
  }
  else
  {
    ROS_ERROR("Not connected to joint trajectory action server");
  }

  schedule_gait_action_server =
      new ScheduleGaitActionServer(n, ActionNames::schedule_gait, &scheduleGaitCallback, false);
  schedule_gait_action_server->start();

  ros::spin();

  return 0;
}
