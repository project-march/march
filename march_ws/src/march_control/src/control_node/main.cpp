// Copyright 2018 Project March.
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>
#include "ros/ros.h"
#include "main.h"
#include "../../../march_main/src/public/communication/TopicNames.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <march_custom_msgs/GaitStatus.h>
#include <fstream>
#include <iostream>

#include <ros/package.h>

#include <boost/algorithm/string.hpp>

void gaitInputCallback(const march_custom_msgs::GaitStatus msg) {
    ROS_INFO("received gait movement message of gait %s", msg.gait_name.c_str());
}

std_msgs::Float64 createMsg(float data) {
    std_msgs::Float64 msg;
    msg.data = data * 1;
    return msg;
}

ros::Publisher left_hip_position_pub;
ros::Publisher left_knee_position_pub;
ros::Publisher left_ankle_position_pub;
ros::Publisher right_hip_position_pub;
ros::Publisher right_knee_position_pub;
ros::Publisher right_ankle_position_pub;


void publishPositions(std::vector<std::string> pos) {
    left_hip_position_pub.publish(createMsg(stof(pos.at(0))));
    left_knee_position_pub.publish(createMsg(stof(pos.at(1))));
    left_ankle_position_pub.publish(createMsg(stof(pos.at(2))));
    right_hip_position_pub.publish(createMsg(stof(pos.at(3))));
    right_knee_position_pub.publish(createMsg(stof(pos.at(4))));
    right_ankle_position_pub.publish(createMsg(stof(pos.at(5))));
}


void playGaitFile(const std::string &fileName, ros::Rate rate) {
    ROS_WARN_STREAM(fileName);

    std::string package_path = ros::package::getPath("march_control");
    ROS_WARN_STREAM(package_path);

    std::string path = package_path + "/src/control_node/gaits/" + fileName;

    ROS_WARN_STREAM(path);
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("File not open");
    }
    std::string line;

    std::getline(file, line);

    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        if (std::getline(file, line)) {
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of("~"));
            publishPositions(strs);

        } else {
            return;
        }
    }
}

void playWalkingAnimation(ros::Rate rate) {
    playGaitFile("left_leg.txt", rate);
    playGaitFile("left_leg.txt", rate);
    playGaitFile("right_swing.txt", rate);
    playGaitFile("left_swing.txt", rate);
    playGaitFile("right_swing.txt", rate);
    playGaitFile("left_step_close.txt", rate);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;

    ros::Subscriber sub_gait_input = n.subscribe(TopicNames::gait_movement, 1000, gaitInputCallback);
    left_hip_position_pub = n.advertise<std_msgs::Float64>(TopicNames::left_hip_position, 1000);
    left_knee_position_pub = n.advertise<std_msgs::Float64>(TopicNames::left_knee_position, 1000);
    left_ankle_position_pub = n.advertise<std_msgs::Float64>(TopicNames::left_ankle_position, 1000);
    right_hip_position_pub = n.advertise<std_msgs::Float64>(TopicNames::right_hip_position, 1000);
    right_knee_position_pub = n.advertise<std_msgs::Float64>(TopicNames::right_knee_position, 1000);
    right_ankle_position_pub = n.advertise<std_msgs::Float64>(TopicNames::right_ankle_position, 1000);

    ros::Rate rate(200);
    std::string package_path = ros::package::getPath("march_control");
    ROS_INFO_STREAM(package_path);

    playWalkingAnimation(rate);

    return 0;
}

