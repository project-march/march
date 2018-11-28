cmake_minimum_required(VERSION 2.8.3)
project(march_control)

add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        controller_manager
        effort_controllers
        joint_state_controller
        march_custom_msgs
        robot_state_publisher
        roslint
        )

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})

catkin_package(
        CATKIN_DEPENDS
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(control_node src/control_node/main.cpp)
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node ${catkin_LIBRARIES})
