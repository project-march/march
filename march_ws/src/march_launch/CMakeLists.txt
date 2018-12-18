cmake_minimum_required(VERSION 2.8.3)
project(march_launch)

find_package(catkin REQUIRED COMPONENTS
        march_control
        march_description
        march_gazebo
        march_state_machine
        march_state_machine_service
        )


catkin_package()
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
