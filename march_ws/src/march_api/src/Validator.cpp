//
// Created by ishadijcks on 26-11-18.
//
#include <random>
#include "Validator.h"

Result Validator::checkURDF(std::string fileName)
{
  //    TODO implement
  return Result(true, "The URDF is valid.");
}

Result Validator::checkXml()
{
  //    TODO implement
  return Result(true, "The xml is valid.");
}

Result Validator::checkConfig()
{
  //    TODO implement
  return Result(true, "The config is valid.");
}