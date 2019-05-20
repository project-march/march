// Copyright 2018 Project March.

#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/algorithm/string.hpp>
#include <trajectory_msgs/JointTrajectory.h>

#include <march_gait_scheduler/Scheduler.h>
#include <march_shared_resources/GaitAction.h>
#include <march_shared_resources/GaitGoal.h>
#include <march_shared_resources/TopicNames.h>

typedef actionlib::SimpleActionServer<march_shared_resources::GaitAction>
    ServerFollowJoint;
//ros::Publisher joint_trajectory_pub;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* followJointTrajectoryAction;
actionlib_msgs::GoalStatus trajectory_status;
Scheduler scheduler;

/**
 * To determine if a goal execution is finished and isn't in progress anymore.
 * @param status the status
 * @return true if the execution is finished if still in progress return false.
 */
bool statusIsTerminal(const actionlib_msgs::GoalStatus &status) {
  switch (status.status) {
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


void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Error?: %i", result->error_code);
  ROS_INFO("Error?: %s", result->error_string.c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback timestamp %d", feedback->header.stamp.toSec());
}

/**
 * Make ros control execute the joint trajectory by setting the trajectory goal
 * @param goal the trajectory goal for ros control
 * @param server The server, which is needed in the callback to return succeeded
 * or aborted.
 */
void executeFollowJointTrajectory(
    const march_shared_resources::GaitGoalConstPtr &goal,
    ServerFollowJoint *server) {

  ROS_INFO("Gait Scheduler received msg to schedule: %s of %s",
           goal->current_subgait.name.c_str(), goal->name.c_str());
  try {
    control_msgs::FollowJointTrajectoryGoal trajectoryMsg =
        scheduler.scheduleTrajectory(goal.get(), ros::Duration().fromSec(0));
    followJointTrajectoryAction->sendGoal(trajectoryMsg, &doneCb, &activeCb, &feedbackCb);
  } catch (std::runtime_error error) {
    ROS_ERROR(
        "When trying to schedule a trajectory the following error occurred: %s",
        error.what());
    ROS_ERROR("Gait scheduling ABORTED");
    server->setAborted();
    return;
  }

  trajectory_status = actionlib_msgs::GoalStatus();
  while (!statusIsTerminal(trajectory_status)) {
    ros::Rate r(100);
    r.sleep();
  }

  if (trajectory_status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
    ROS_INFO("Gait scheduling SUCCEEDED");
    server->setSucceeded();
    return;
  } else {
    ROS_ERROR("Gait scheduling ABORTED");
    server->setAborted();
    return;
  }
}

/**
 * This callback updates the status of the current trajectory execution of ros
 * control.
 * This status is needed to determine if the execution was successful.
 * @param status_message the status message of ros control
 */
void TrajectoryExecutionStatusCallback(
    const control_msgs::FollowJointTrajectoryActionFeedbackConstPtr
        &status_message) {
  trajectory_status = actionlib_msgs::GoalStatus();
  trajectory_status.status = status_message->status.status;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gait_scheduler_node");
  ros::NodeHandle n;
  ros::Rate rate(100);

//  joint_trajectory_pub =
//      n.advertise<control_msgs::FollowJointTrajectoryActionGoal>(
//          TopicNames::follow_joint_trajectory_execution, 1000);
  followJointTrajectoryAction = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/march/trajectory_controller/follow_joint_trajectory", true);
  ROS_INFO("Waiting for followJointTrajectoryAction server to start.");
  // wait for the action server to start
  followJointTrajectoryAction->waitForServer(); //will wait for infinite time

  ros::Subscriber joint_trajectory_status_sub =
      n.subscribe(TopicNames::follow_joint_trajectory_execution_states, 1000,
                  TrajectoryExecutionStatusCallback);

  ServerFollowJoint server_follow_joint_trajectory(
      n, ActionNames::schedule_gait,
      boost::bind(&executeFollowJointTrajectory, _1,
                  &server_follow_joint_trajectory),
      false);

  server_follow_joint_trajectory.start();

  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
