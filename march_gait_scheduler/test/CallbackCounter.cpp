// Copyright 2019 Project March.

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/Temperature.h>

struct CallbackCounter
{
  CallbackCounter() : count(0)
  {
  }

  void cb_test(const sensor_msgs::Temperature& msg)
  {
    ++count;
  }

  void cb(const control_msgs::FollowJointTrajectoryAction& msg)
  {
    ++count;
  }

  void cb_action(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal){
    ++count;
  }

  void cb_fake(){
    ++count;
  }

  int count;
};