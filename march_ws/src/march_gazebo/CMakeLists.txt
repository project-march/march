cmake_minimum_required(VERSION 2.8.3)
project(march_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  march_control
  march_description
  xacro
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
