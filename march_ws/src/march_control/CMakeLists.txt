cmake_minimum_required(VERSION 2.8.3)
project(march_control)


add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        controller_manager
        joint_state_controller
        robot_state_publisher
        rqt_gui
        march_custom_msgs
        effort_controllers
        )

catkin_package(
        CATKIN_DEPENDS march_custom_msgs effort_controllers

)
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(control_node src/control_node/main.cpp)
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node ${catkin_LIBRARIES})
