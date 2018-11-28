// Copyright 2018 Project March.

#ifndef PROJECT_VALIDATOR_H
#define PROJECT_VALIDATOR_H

#include "common/Result.h"

class Validator
{
public:
  /**
   * Check whether or not the URDF files are configured correctly.
   * @return True if the URDF files are configured correctly, False if they are not.
   */
  static Result checkURDF(std::string fileName);
  static Result checkConfig();
  static Result checkXml();
};

#endif  // PROJECT_VALIDATOR_H
