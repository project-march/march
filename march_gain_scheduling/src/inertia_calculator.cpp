#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <sstream>
#include <urdf/model.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char** argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "inertia_calculator");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
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
  if (!tree.getChain("joint_base", "bar", chain))
  {
    ROS_ERROR("Failed to extract chain from 'joint_base' to 'bar' in kdl tree");
    return false;
  }
  ROS_INFO("Extracted chain from kdl tree");

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    KDL::ChainDynParam dyn(chain, KDL::Vector::Zero());

    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntSpaceInertiaMatrix H(chain.getNrOfJoints());
    dyn.JntToMass(q, H);

    for (unsigned int i = 0; i < 8; i++)
    {
      inertias.data[i] = chain.getSegment(i).getInertia();
    }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(inertias);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}