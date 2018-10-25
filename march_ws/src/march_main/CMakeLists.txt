cmake_minimum_required(VERSION 2.8.3)
project(march_main)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        nav_msgs
        roscpp
        )


add_definitions(-std=c++11 -Wall -g)


catkin_package(
        #  INCLUDE_DIRS include
        #LIBRARIES aseia
        CATKIN_DEPENDS roscpp std_msgs nav_msgs
        #  DEPENDS aseia
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(march_main src/march_main_node/main.cpp src/march_main_node/Cube.cpp)

#   src/${PROJECT_NAME}/aseia_base.cpp
# )

## Declare a cpp executable
add_executable(Exoskeleton src/march_main_node/main.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(car march_main march_main_generate_messages_cpp)


target_link_libraries(Exoskeleton ${catkin_LIBRARIES} march_main)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}-test launch/gtest.test test/cube_test.cpp)
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_main)
endif ()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
