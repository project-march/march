// Copyright 2019 Project March.
#include "gtest/gtest.h"
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
// TODO(Tim) Make this a normal import
//#include <march_gait_scheduler/Scheduler.h>
#include "../src/Scheduler.cpp"

class DelayGaitTest : public ::testing::Test
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
};

TEST_F(DelayGaitTest, SetTimeToCurrent)
{
  ros::Time::init();
  trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
  trajectory.joint_names = { "joint1" };
  trajectory.points = { createPoint() };
  ros::Time current_time = ros::Time::now();
//  Scheduler::setStartTimeGait(trajectory, current_time);
//  ASSERT_EQ(current_time.toSec(), trajectory.header.stamp.toSec());
}
