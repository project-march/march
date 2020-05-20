#include <actionlib/server/simple_action_server.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

#ifndef MARCH_WS_INERTIA_CALCULATOR_NODE_H
#define MARCH_WS_INERTIA_CALCULATOR_NODE_H

class inCalcClass
{
private:
  double joint_pos_[8];
  double joint_vel_[8];
  ros::NodeHandle n_;

public:
  void joint_trajectory_feedback_callback(const sensor_msgs::JointStateConstPtr& msg)
  {
    for (int i = 0; i < 8; i++)
    {
      joint_pos_[i] = msg->position[i];
      joint_vel_[i] = msg->velocity[i];
    }
  }
  inCalcClass(ros::NodeHandle n)
  {
    for (int i = 0; i < 8; i++)
    {
      joint_pos_[i] = 0.0;
      joint_vel_[i] = 0.0;
    }
    n_ = n;
  }
  float getPos(int idx)
  {
    return joint_pos_[idx];
  }
  float getVel(int idx)
  {
    return joint_vel_[idx];
  }
};

#endif  // MARCH_WS_INERTIA_CALCULATOR_NODE_H
