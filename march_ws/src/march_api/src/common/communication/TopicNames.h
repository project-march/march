// Copyright 2018 Project March.
#ifndef MARCH_MAIN_TOPICNAMES_H
#define MARCH_MAIN_TOPICNAMES_H

#include <string>
namespace TopicNames
{
const char * const gait_input = "master/gait_input";
const char * const play_input = "master/play_input";
const char * const gait_status = "gait/status";
const char * const gait_movement = "gait/movement";
const char * const left_hip_position = "march/left_hip_position_controller/command";
const char * const left_knee_position = "march/left_knee_position_controller/command";
const char * const left_ankle_position = "march/left_ankle_position_controller/command";
const char * const right_hip_position = "march/right_hip_position_controller/command";
const char * const right_knee_position = "march/right_knee_position_controller/command";
const char * const right_ankle_position = "march/right_ankle_position_controller/command";
};  // namespace TopicNames

namespace ServiceNames
{
const char * const service_prefix = "march/";
const char * const validation = "march/launch_validation";
};

#endif  // MARCH_MAIN_TOPICNAMES_H
