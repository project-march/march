cmake_minimum_required(VERSION 3.4)
project(march_cmake)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS march_cmake-extras.cmake)

## Install all cmake files
install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)