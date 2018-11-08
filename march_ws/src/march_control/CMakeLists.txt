cmake_minimum_required(VERSION 2.8.3)
project(march_control)

find_package(catkin REQUIRED COMPONENTS
        controller_manager
        joint_state_controller
        robot_state_publisher
        rqt_gui
        )

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
