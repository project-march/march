//
// Created by tim on 23-10-18.
//

#include "march_main_node.h"

int main(int argc, char **argv) {
    std::cout << "test";
    ros::init(argc,argv,"Henk");
    ros::NodeHandle n;
    ros::Publisher hello_pub = n.advertise<std_msgs::String>("greeting", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        hello_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }


    return 0;
}
