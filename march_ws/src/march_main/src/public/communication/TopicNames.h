// Copyright 2018 Project March.
#ifndef MARCH_MAIN_TOPICNAMES_H
#define MARCH_MAIN_TOPICNAMES_H

#include <string>
namespace TopicNames
{
static const std::string gait_input = "master/gait_input";
static const std::string play_input = "master/play_input";

static const std::string gait_status = "gait/status";
static const std::string gait_movement = "gait/movement";

static const std::string joint_position = "control/joint_position";
};

namespace ServiceNames
{
static const std::string gait_input = "input/gait_input";
static const std::string play_input = "input/play_input";
};

#endif  // MARCH_MAIN_TOPICNAMES_H
