cmake_minimum_required(VERSION 2.8.3)
project(march_main)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        march_custom_msgs
        message_generation
        nav_msgs
        roscpp
        roslint
        rostest
        std_msgs
        )

add_definitions(-std=c++11 -Wall -g)

catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES aseia
        CATKIN_DEPENDS nav_msgs march_custom_msgs message_runtime roscpp std_msgs
        # DEPENDS aseia
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

set(sourcefiles
        src/march_main_node/main.cpp
        src/march_main_node/Cube.cpp
        src/public/enum/gait_enum.h
        )

## Declare a cpp library
add_library(march_main ${sourcefiles})

#   src/${PROJECT_NAME}/aseia_base.cpp
# )

## Declare a cpp executable
add_executable(Exoskeleton src/march_main_node/main.cpp)
add_executable(MastNode src/master_node/master_node.cpp)
add_executable(DeveloperInputNode src/developer_input_node.cpp)
add_executable(DataLoggerNode src/data_logger.cpp)
add_executable(GaitControllerNode src/gait_controller_node.cpp)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_dependencies(MastNode ${catkin_EXPORTED_TARGETS})
add_dependencies(DeveloperInputNode ${catkin_EXPORTED_TARGETS})
add_dependencies(DataLoggerNode ${catkin_EXPORTED_TARGETS})
add_dependencies(GaitControllerNode ${catkin_EXPORTED_TARGETS})

target_link_libraries(Exoskeleton ${catkin_LIBRARIES} march_main)
target_link_libraries(MastNode ${catkin_LIBRARIES} march_main)
target_link_libraries(DeveloperInputNode ${catkin_LIBRARIES} march_main)
target_link_libraries(DataLoggerNode ${catkin_LIBRARIES} march_main)
target_link_libraries(GaitControllerNode ${catkin_LIBRARIES} march_main)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    #    rostest is already imported earlier
    #    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}-test launch/gtest.test test/cube_test.cpp)
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_main)
endif ()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
