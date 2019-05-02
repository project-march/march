// Copyright 2018 Project March.

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>

#include <boost/algorithm/string.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <march_shared_resources/GaitNameAction.h>
#include <march_shared_resources/TopicNames.h>
#include <march_rqt_gait_generator/MarchGait.h>
#include <march_gait_scheduler/Scheduler.h>

typedef actionlib::SimpleActionServer<march_shared_resources::GaitNameAction> ServerGaitName;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ServerFollowJoint;
ros::Publisher joint_trajectory_pub;
actionlib_msgs::GoalStatus trajectory_status;

/**
 * To determine if a goal execution is finished and isn't in progress anymore.
 * @param status the status
 * @return true if the execution is finished if still in progress return false.
 */
bool statusIsTerminal(const actionlib_msgs::GoalStatus& status)
{
  switch (status.status)
  {
    case actionlib_msgs::GoalStatus::PREEMPTED:
      return true;
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      return true;
    case actionlib_msgs::GoalStatus::ABORTED:
      return true;
    case actionlib_msgs::GoalStatus::REJECTED:
      return true;
    case actionlib_msgs::GoalStatus::RECALLED:
      return true;
  }
  return false;
}


/**
 * Make ros control execute the joint trajectory by setting the trajectory goal
 * @param goal the trajectory goal for ros control
 * @param server The server, which is needed in the callback to return succeeded or aborted.
 */
void executeFollowJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal,
                                  ServerFollowJoint* server)
{
  control_msgs::FollowJointTrajectoryActionGoal trajectoryMsg;
//  Scheduler::delayTrajectory(goal->trajectory);
  trajectoryMsg.goal = *goal;
  joint_trajectory_pub.publish(trajectoryMsg);
  trajectory_status = actionlib_msgs::GoalStatus();

  while (!statusIsTerminal(trajectory_status))
  {
    ros::Rate r(100);
    r.sleep();
  }

  if (trajectory_status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
  {
    ROS_INFO("executeFollowJointTrajectory: Trajectory SUCCEEDED");
    server->setSucceeded();
    return;
  }
  else
  {
    ROS_ERROR("executeFollowJointTrajectory: Trajectory ABORTED");
    server->setAborted();
    return;
  }
}

/**
 * This callback updates the status of the current trajectory execution of ros control.
 * This status is needed to determine if the execution was successful.
 * @param status_message the status message of ros control
 */
void TrajectoryExecutionStatusCallback(const control_msgs::FollowJointTrajectoryActionFeedbackConstPtr& status_message)
{
  trajectory_status = actionlib_msgs::GoalStatus();
  trajectory_status.status = status_message->status.status;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_scheduler_node");
  ros::NodeHandle n;

  ros::Rate rate(100);
  std::string package_path = ros::package::getPath("gait_scheduler_node");

  std::string planning_group;

  //  retrieving planning group name (initialized in launch file) from parameter server
  n.getParam("/control_node/planning_group", planning_group);

  //  determining which planning group is used. If test_joint, topic name for ServerFollowJoint is changed

  joint_trajectory_pub =
      n.advertise<control_msgs::FollowJointTrajectoryActionGoal>(TopicNames::follow_joint_trajectory_execution, 1000);

  ros::Subscriber joint_trajectory_status_sub =
      n.subscribe(TopicNames::follow_joint_trajectory_execution_states, 1000, TrajectoryExecutionStatusCallback);

  ServerFollowJoint server_follow_joint_trajectory(
      n, ActionNames::schedule_gait,
      boost::bind(&executeFollowJointTrajectory, _1, &server_follow_joint_trajectory), false);
  server_follow_joint_trajectory.start();

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
