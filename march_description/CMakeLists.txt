cmake_minimum_required(VERSION 2.8.3)
project(march_description)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  rviz
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
