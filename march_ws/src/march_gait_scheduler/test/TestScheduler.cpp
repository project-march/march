// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <march_gait_scheduler/Scheduler.h>

class SchedulerTest : public ::testing::Test {
 protected:
};

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

TEST_F(SchedulerTest, DelayOnePointTrajectory) {
  int time_from_start_first_point = 0;

  trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
  trajectory.joint_names = {"joint1"};
  trajectory.points = {createPoint(time_from_start_first_point)};
  ros::Duration duration = ros::Duration().fromNSec(20);
  Scheduler::delayTrajectory(trajectory, duration);

  ASSERT_EQ(time_from_start_first_point + 20, trajectory.points[0].time_from_start);
}

TEST_F(SchedulerTest, DelayTwoPointTrajectory) {
  int time_from_start_first_point = 0;
  int time_from_start_second_point = 10;

  trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
  trajectory.joint_names = {"joint1"};
  trajectory.points = {createPoint(time_from_start_first_point), createPoint(0.5,0,0,time_from_start_second_point)};
  ros::Duration duration = ros::Duration().fromNSec(20);
  Scheduler::delayTrajectory(trajectory, duration);

  ASSERT_EQ(time_from_start_first_point + 20, trajectory.points[0].time_from_start);
  ASSERT_EQ(time_from_start_second_point + 20, trajectory.points[1].time_from_start);
}


TEST_F(SchedulerTest, ) {
  int time_from_start_first_point = 10;

  trajectory_msgs::JointTrajectory trajectory = trajectory_msgs::JointTrajectory();
  trajectory.joint_names = {"joint1"};
  trajectory.points = {createPoint(time_from_start_first_point)};
  ros::Duration duration = ros::Duration().fromNSec(20);
  Scheduler::delayTrajecgit update sutory(trajectory, duration);

}

/**
 * The main method which runs all the tests
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "testnode");
  testing::InitGoogleTest(&argc, argv);

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
