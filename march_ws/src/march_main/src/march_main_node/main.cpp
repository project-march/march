//
// Created by tim on 23-10-18.
//

#include "main.h"

int main(int argc, char **argv) {

  std::cout << "test";
  ros::init(argc, argv, "adele");
  ros::NodeHandle n;

  ros::Publisher hello_pub = n.advertise<std_msgs::String>("talking", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    std_msgs::String msg;
    msg.data = "hello";
    ROS_INFO("%s", msg.data.c_str());
    hello_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
