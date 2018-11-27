cmake_minimum_required(VERSION 2.8.3)
project(march_api)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        march_custom_msgs
        march_control
        message_generation
        roscpp
        rospy
        roslint
        rostest
        rqt_gui
        rviz
        std_msgs
        )

add_definitions(-std=c++11 -Wall -g)


add_message_files(
        FILES
        Gait.msg
        GaitStatus.msg
        GaitInputMaster.msg
        PlayInputMaster.msg
)

add_service_files(
        FILES
        GaitInstruction.srv
        GaitInput.srv
        PlayInput.srv
        Trigger.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)


catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES aseia
        CATKIN_DEPENDS
        march_custom_msgs
        march_control
        roscpp
        rospy
        rqt_gui
        std_msgs

        # DEPENDS aseia
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

set(sourcefiles
        src/api_node/main.cpp
        src/Validator.cpp
        src/Validator.h

        )
# )


#add_library(march_api ${sourcefiles})

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
        src/api_node/main.cpp
        src/Validator.cpp
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
    #    add_rostest_gtest(${PROJECT_NAME}-test launch/gtest.test
    #            test/cube_test.cpp
    #            )
    #    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_api)
endif ()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
