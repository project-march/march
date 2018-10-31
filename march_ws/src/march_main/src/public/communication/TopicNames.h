// Copyright 2018 Project March.
#ifndef MARCH_MAIN_TOPICNAMES_H
#define MARCH_MAIN_TOPICNAMES_H

#include <string>
namespace TopicNames
{
const static std::string gait_input = "input/gait_input";
const static std::string play_input = "input/gait_input";

const static std::string gait_status = "/gait/status";
const static std::string gait_movement = "/gait/movement";
};
namespace ServiceNames
{
const static std::string gait_input = "master/gait_input";
const static std::string play_input = "master/gait_input";
};

#endif  // MARCH_MAIN_TOPICNAMES_H
