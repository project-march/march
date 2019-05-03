cmake_minimum_required(VERSION 2.8.3)
project(march_gait_scheduler)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        control_msgs
        march_shared_resources
        roscpp
        roslint
        rostest
        trajectory_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(sourcefiles
        src/GaitSchedulerNode.cpp
        include/march_gait_scheduler/Scheduler.h
        src/Scheduler.cpp
        )

add_library(march_gait_scheduler ${sourcefiles})

add_executable(gait_scheduler_node
        src/GaitSchedulerNode.cpp
        src/Scheduler.cpp
        include/march_gait_scheduler/Scheduler.h
        )
add_dependencies(gait_scheduler_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gait_scheduler_node ${catkin_LIBRARIES})

#############
## Testing ##
#############


## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    add_rostest_gmock(${PROJECT_NAME}-test
            test/main-test.test
            test/TestRunner.cpp
            test/ScheduleOneGaitTest.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_gait_scheduler gtest gmock)
endif ()
