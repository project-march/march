cmake_minimum_required(VERSION 3.0.2)
project(march_launch)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
