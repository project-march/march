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


## Add gtest based cpp test target and link libraries
if(TARGET ${PROJECT_NAME}-test)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}-test launch/gtest.test test/gtest.cpp)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
