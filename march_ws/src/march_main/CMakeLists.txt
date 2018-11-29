cmake_minimum_required(VERSION 2.8.3)
project(march_main)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        march_control
        march_custom_msgs
        roscpp
        roslint
        rospy
        rostest
        rqt_gui
        rviz
        )

add_definitions(-std=c++11 -Wall -g)


catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES aseia
         CATKIN_DEPENDS
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

set(sourcefiles
        src/Validator.cpp
        src/Validator.h
        api/main.cpp
        )

add_library(march_main ${sourcefiles})

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})

## Declare a cpp executable
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_executable(api_node
        src/Validator.cpp
        api/main.cpp
        api/LaunchAPI.cpp
        api/MovementAPI.cpp
        )

add_dependencies(api_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(api_node ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    #    rostest is already imported earlier
    #    find_package(rostest REQUIRED)
    #    add_rostest_gtest(${PROJECT_NAME}-test launch_sm/gtest.test
    #            test/cube_test.cpp
    #            )
    #    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_main)
endif ()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
