// Copyright 2018 Project March.

#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "../Validator.h"
#include "march_api/Trigger.h"


bool validator(march_api::Trigger::Request &request,
                       march_api::Trigger::Response &response)
{
    ROS_INFO("URDF validator called");

    /**
     * TODO lookup folder with all URDF files.
     * Loop through all files and perfows the check for each file
     */

    Result urdf = Validator::checkURDF("dummy_file");
    if (!urdf.success) {
        ROS_WARN("Invalid URDF");
        response.success = false;
        response.message = urdf.message;
        return true;
    }
    ROS_INFO("Hardware validator called");

    Result hardware = Validator::checkHardware();
    if (!hardware.success) {
        ROS_WARN("Invalid Hardware");
        response.success = false;
        response.message = hardware.message;
        return true;
    }

    response.success = true;
    response.message = "All validations passed!";
    return true;
}


int main(int argc, char** argv)
{

    ROS_INFO("Starting the api node...");
    ros::init(argc, argv, "api_node");
    ros::NodeHandle n;

    ros::ServiceServer xml_validator = n.advertiseService("march/launch_validation", validator);

    ROS_INFO("Success!");
    ros::Rate rate(100);
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
