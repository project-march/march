// Copyright 2018 Project March.

#ifndef PROJECT_LAUNCH_API_H
#define PROJECT_LAUNCH_API_H

#include <march_main/Trigger.h>

class LaunchAPI
{
public:
  /**
   * Returns true if the config is valid.
   * @srv march_main/Trigger.srv
   * srv.success is true iff the config is valid.
  */
  static bool urdf_validator(march_main::Trigger::Request& request, march_main::Trigger::Response& response);

  static bool config_validator(march_main::Trigger::Request& request, march_main::Trigger::Response& response);

  static bool xml_validator(march_main::Trigger::Request& request, march_main::Trigger::Response& response);
};

#endif  // PROJECT_LAUNCH_API_H
