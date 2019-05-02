// Copyright 2019 Project March.
#include "gtest/gtest.h"
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

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

TEST_F(DelayGaitTest, Delay20NSec)
{
    ros::Time::init();
    trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
    trajectory.joint_names = {"joint1"};
    trajectory.points = {createPoint()};
    ros::Duration duration = ros::Duration().fromNSec(20);
//    ASSERT_GT(ros::Time::now().toNSec() + 1000, trajectory.header.stamp.toNSec());
}
