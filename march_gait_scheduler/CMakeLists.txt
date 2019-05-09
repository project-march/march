cmake_minimum_required(VERSION 2.8.3)
project(march_gait_scheduler)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        control_msgs
        controller_manager
        effort_controllers
        joint_state_controller
        march_shared_resources
        robot_state_publisher
        roslint
        rostest
        trajectory_msgs
        )

catkin_package(
        CATKIN_DEPENDS
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

set(sourcefiles
        src/GaitSchedulerNode.cpp
        )

add_library(march_gait_scheduler ${sourcefiles})


# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(gait_scheduler_node
        src/GaitSchedulerNode.cpp
        )
add_dependencies(gait_scheduler_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gait_scheduler_node ${catkin_LIBRARIES})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(${PROJECT_NAME}-test test/simple-test.test
            test/ExampleTest.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_gait_scheduler)
endif ()

