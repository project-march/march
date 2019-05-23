cmake_minimum_required(VERSION 2.8.3)
project(march_gait_scheduler)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        code_coverage
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

add_library(march_gait_scheduler
        src/GaitSchedulerNode.cpp
        include/march_gait_scheduler/Scheduler.h
        src/Scheduler.cpp
)

target_link_libraries(march_gait_scheduler ${catkin_LIBRARIES})

add_executable(gait_scheduler_node src/GaitSchedulerNode.cpp)

target_link_libraries(gait_scheduler_node march_gait_scheduler)
add_dependencies(gait_scheduler_node ${catkin_EXPORTED_TARGETS})

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        APPEND_COVERAGE_COMPILER_FLAGS()
    endif()

    add_rostest_gmock(${PROJECT_NAME}-test
            test/main-test.test
            test/TestRunner.cpp
            test/ScheduleOneGaitTest.cpp
            test/ScheduleMultipleGaitsTest.cpp
            test/EndTimeTest.cpp
            test/AcceptanceTest.cpp
            test/CallbackCounter.cpp
            )

    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_gait_scheduler)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
                NAME ${PROJECT_NAME}_coverage
                DEPENDS tests
        )
    endif()
endif ()
