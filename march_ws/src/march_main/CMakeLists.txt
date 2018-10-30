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
        src/public/enum/gait_enum.h
        src/master_node/main.cpp
        )
#   src/${PROJECT_NAME}/aseia_base.cpp
# )

## Declare a cpp executable
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_executable(master_node src/master_node/main.cpp)
add_dependencies(master_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(master_node ${catkin_LIBRARIES})

add_executable(developer_inputNode src/developer_input_node/main.cpp)
add_dependencies(developer_inputNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(developer_inputNode ${catkin_LIBRARIES})

add_executable(datalogger_node src/datalogger_node/main.cpp)
add_dependencies(datalogger_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(datalogger_node ${catkin_LIBRARIES})

add_executable(gait_controller_node src/gait_node/main.cpp)
add_dependencies(gait_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gait_controller_node ${catkin_LIBRARIES})

add_executable(control_node src/control_node/main.cpp)
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node ${catkin_LIBRARIES})

add_executable(sensor_node src/sensor_node/main.cpp src/sensor_node/main.h)
add_dependencies(sensor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_node ${catkin_LIBRARIES})
# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include" "--linelength=120")
file(GLOB_RECURSE LINT_FILES
        "src/*/src/*.h" "src/*/src/*.cpp")
roslint_cpp(${LINT_FILES})


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
