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
ServerFollowJoint *schedule_gait_action_server;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    *followJointTrajectoryAction;
actionlib_msgs::GoalStatus trajectory_status;
Scheduler scheduler;

void doneCb(const actionlib::SimpleClientGoalState &state,
            const control_msgs::FollowJointTrajectoryResultConstPtr &result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Error?: %i", result->error_code);
  ROS_INFO("Error?: %s", result->error_string.c_str());
  ROS_INFO("Gait scheduling SUCCEEDED");
}

// Called once when the goal becomes active
void activeCb() { ROS_DEBUG("Goal just went active"); }

// Called every time feedback is received for the goal
void feedbackCb(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback) {
  if (scheduler.getEndTimeCurrentGait().toSec() -
          feedback->header.stamp.toSec() <
      scheduler.APPROVE_TIME_BEFORE_END_GAIT) {
    if (schedule_gait_action_server->isActive()) {
      schedule_gait_action_server->setSucceeded();
      ROS_INFO_THROTTLE(1, "Set SUCCEEDED!!!!");
    }
  }
}

/**
 * Make ros control execute the joint trajectory by setting the trajectory goal
 * @param goal the trajectory goal for ros control
 * @param server The server, which is needed in the callback to return succeeded
 * or aborted.
 */
void executeFollowJointTrajectory(
    const march_shared_resources::GaitGoalConstPtr &goal) {

  ROS_INFO("Gait Scheduler received msg to schedule: %s of %s",
           goal->current_subgait.name.c_str(), goal->name.c_str());
  try {
    control_msgs::FollowJointTrajectoryGoal trajectoryMsg =
        scheduler.scheduleTrajectory(goal.get(), ros::Duration().fromSec(0));
    followJointTrajectoryAction->sendGoal(trajectoryMsg, &doneCb, &activeCb,
                                          &feedbackCb);
  } catch (std::runtime_error error) {
    ROS_ERROR(
        "When trying to schedule a trajectory the following error occurred: %s",
        error.what());
    ROS_ERROR("Gait scheduling ABORTED");
    schedule_gait_action_server->setAborted();
    return;
  }

  while (schedule_gait_action_server->isActive()) {
    ros::Rate r(100);
    r.sleep();
    ROS_DEBUG_THROTTLE(1, "Waiting on gait execution");
  }
  return;
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
  followJointTrajectoryAction = new actionlib::SimpleActionClient<
      control_msgs::FollowJointTrajectoryAction>(
      "/march/trajectory_controller/follow_joint_trajectory", true);
  // wait for the action server to start
  followJointTrajectoryAction->waitForServer(); // will wait for infinite time

  ros::Subscriber joint_trajectory_status_sub =
      n.subscribe(TopicNames::follow_joint_trajectory_execution_states, 1000,
                  TrajectoryExecutionStatusCallback);

  schedule_gait_action_server = new ServerFollowJoint(
      n, ActionNames::schedule_gait, &executeFollowJointTrajectory, false);
  schedule_gait_action_server->start();

  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
