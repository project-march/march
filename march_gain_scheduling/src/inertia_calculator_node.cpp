#include "march_gain_scheduling/inertia_calculator_node.h"

#include "ros/ros.h"
#include "march_gain_scheduling/inertia_calculator_node.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/simple_action_server.h>
#include "std_msgs/Float64MultiArray.h"
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <sstream>
#include <urdf/model.h>

#include "boost/scoped_ptr.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "inertia_calculator_node");

  ros::NodeHandle n;

  inCalcClass place(n);

  ros::Subscriber sub =
      n.subscribe("/march/joint_states", 1000, &inCalcClass::joint_trajectory_feedback_callback, &place);
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("inertia_publisher", 1000);
  std_msgs::Float64MultiArray inertias;
  inertias.data.resize(8);

  ros::Rate loop_rate(10);

  urdf::Model urdf_model;
  if (!urdf_model.initParam("/robot_description"))
  {
    ROS_ERROR("Failed to parse urdf model from robot description");
    return false;
  }
  ROS_INFO("Parsed urdf model from robot description");

  // Compute the KDL tree of the robot from the URDF.
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree))
  {
    ROS_ERROR("Failed to parse kdl tree from urdf model");
    return false;
  }
  ROS_INFO("Parsed kdl tree from urdf model");

  // Extract chain from KDL tree. This from base to end effector
  KDL::Chain chain;
  if (!tree.getChain("hip_base", "foot_left", chain))
  {
    ROS_ERROR("Failed to extract chain from 'hip_base' to 'foot_left' in kdl tree");
    return false;
  }
  ROS_INFO("Extracted chain from kdl tree");

  KDL::JntArray q(chain.getNrOfJoints());
  while (ros::ok())
  {
    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
    {
      q.data[i] = place.getPos(i);
    }

    KDL::ChainDynParam dyn(chain, KDL::Vector::Zero());
    KDL::JntSpaceInertiaMatrix H(chain.getNrOfJoints());
    dyn.JntToMass(q, H);

    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
    {
      inertias.data[i] = H(i, i);
    }

    chatter_pub.publish(inertias);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}