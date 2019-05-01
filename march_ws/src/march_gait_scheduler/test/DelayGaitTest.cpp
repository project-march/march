// Copyright 2019 Project March.
#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include "ros/ros.h"
#include <march_rqt_gait_generator/MarchGait.h>
#include <march_gait_scheduler/Scheduler.h>


class DelayGaitTest : public ::testing::Test
{
 protected:

  trajectory_msgs::JointTrajectoryPoint createPoint(double position = 0,
                                                    double velocity = 0,
                                                    double acceleration = 0,
                                                    int time_from_start = 0) {
    trajectory_msgs::JointTrajectoryPoint point = trajectory_msgs::JointTrajectoryPoint();
    point.positions = {position};
    point.velocities = {velocity};
    point.accelerations = {acceleration};
    point.time_from_start = ros::Duration().fromNSec(time_from_start);
    return point;
  }

};


TEST_F(DelayGaitTest, DelayOnePointTrajectory) {
  int time_from_start_first_point = 0;

  trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
  trajectory.joint_names = {"joint1"};
  trajectory.points = {createPoint(time_from_start_first_point)};
  ros::Duration duration = ros::Duration().fromNSec(20);
  march_rqt_gait_generator::MarchGait marchGait = march_rqt_gait_generator::MarchGait();
  marchGait.joint_trajectory = trajectory;
  Scheduler::delayGait(marchGait, duration);

  ASSERT_EQ(time_from_start_first_point + 20, trajectory.points[0].time_from_start.toNSec());
}

TEST_F(DelayGaitTest, DelayTwoPointTrajectory) {
  int time_from_start_first_point = 0;
  int time_from_start_second_point = 10;

  trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
  trajectory.joint_names = {"joint1"};
  trajectory.points = {createPoint(time_from_start_first_point), createPoint(0.5,0,0,time_from_start_second_point)};
  ros::Duration duration = ros::Duration().fromNSec(20);
  march_rqt_gait_generator::MarchGait marchGait = march_rqt_gait_generator::MarchGait();
  marchGait.joint_trajectory = trajectory;
  Scheduler::delayGait(marchGait, duration);

  ASSERT_EQ(time_from_start_first_point + 20, trajectory.points[0].time_from_start.toNSec());
  ASSERT_EQ(time_from_start_second_point + 20, trajectory.points[1].time_from_start.toNSec());
}


TEST_F(DelayGaitTest, DurationNotEffected) {
  int time_from_start_first_point = 10;

  trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
  trajectory.joint_names = {"joint1"};
  trajectory.points = {createPoint(time_from_start_first_point)};
  ros::Duration duration = ros::Duration().fromNSec(20);
  march_rqt_gait_generator::MarchGait marchGait = march_rqt_gait_generator::MarchGait();
  marchGait.duration = ros::Duration().fromNSec(42);
  marchGait.joint_trajectory = trajectory;
  Scheduler::delayGait(marchGait, duration);

  ASSERT_EQ(42, marchGait.duration.toNSec());
}
