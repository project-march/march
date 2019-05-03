// Copyright 2019 Project March.
#include "gtest/gtest.h"
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
// TODO(Tim) Make this a normal import
//#include <march_gait_scheduler/Scheduler.h>
#include "../src/Scheduler.cpp"

class ScheduleOneGaitTest : public ::testing::Test
{
protected:
  trajectory_msgs::JointTrajectoryPoint createPoint(double position = 0, double velocity = 0, double acceleration = 0,
                                                    int time_from_start = 0)
  {
    trajectory_msgs::JointTrajectoryPoint point = trajectory_msgs::JointTrajectoryPoint();
    point.positions = { position };
    point.velocities = { velocity };
    point.accelerations = { acceleration };
    point.time_from_start = ros::Duration().fromNSec(time_from_start);
    return point;
  }

  trajectory_msgs::JointTrajectory fake_sit_trajectory()
  {
    march_shared_resources::GaitGoal goal;
    trajectory_msgs::JointTrajectory jointTrajectory;
    jointTrajectory.joint_names = { "left_hip", "left_knee", "left_ankle", "right_hip", "right_knee", "right_ankle" };
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = { 1.3, 1.3, 0.349065850399, 1.3, 1.3, 0.349065850399 };
    point.velocities = { 0, 0, 0, 0, 0, 0 };
    point.time_from_start = ros::Duration().fromSec(3);
    jointTrajectory.points = { point };
    return jointTrajectory;
  }
};

TEST_F(ScheduleOneGaitTest, ScheduleNow)
{
  ros::Time::init();
  ros::Time current_time = ros::Time::now();
  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  control_msgs::FollowJointTrajectoryActionGoal trajectoryMsg =
      scheduler.scheduleTrajectory(&gaitGoalConst, current_time);

  // TODO(TIM) check which header should be set (now only the trajectory header in the message is set, not the high lvl
  // header)
  ASSERT_EQ(current_time.toSec(), trajectoryMsg.goal.trajectory.header.stamp.toSec());
}

TEST_F(ScheduleOneGaitTest, ScheduleInTheFuture)
{
  ros::Time::init();
  ros::Time currentTime = ros::Time::now();
  ros::Time futureTime = currentTime + ros::Duration().fromSec(50);

  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  control_msgs::FollowJointTrajectoryActionGoal trajectoryMsg =
      scheduler.scheduleTrajectory(&gaitGoalConst, futureTime);

  ASSERT_EQ(futureTime.toSec(), trajectoryMsg.goal.trajectory.header.stamp.toSec());
  ASSERT_NE(currentTime.toSec(), trajectoryMsg.goal.trajectory.header.stamp.toSec());
}

TEST_F(ScheduleOneGaitTest, ScheduleInThePast)
{
  ros::Time::init();
  ros::Time currentTime = ros::Time::now();
  ros::Time pastTime = currentTime - ros::Duration().fromSec(50);

  march_shared_resources::GaitGoal gaitGoal;
  gaitGoal.current_subgait.trajectory = fake_sit_trajectory();
  gaitGoal.current_subgait.name = "sit";
  gaitGoal.name = "sit";
  const auto& gaitGoalConst = const_cast<const march_shared_resources::GaitGoal&>(gaitGoal);

  Scheduler scheduler;
  control_msgs::FollowJointTrajectoryActionGoal trajectoryMsg = scheduler.scheduleTrajectory(&gaitGoalConst, pastTime);

  ASSERT_EQ(pastTime.toSec(), trajectoryMsg.goal.trajectory.header.stamp.toSec());
  ASSERT_NE(currentTime.toSec(), trajectoryMsg.goal.trajectory.header.stamp.toSec());
}
