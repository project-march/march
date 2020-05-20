cmake_minimum_required(VERSION 2.8.3)
project(march_gain_scheduling)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    eigen_conversions
    kdl_parser
    march_shared_resources
    roscpp
    rospy
    std_msgs
    urdf
    )

find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    march_shared_resources
    roscpp
    rospy
    std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS} ${Boost_LIBRARY_DIR})

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

if (CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test/one_step_linear_interpolation_test.py)
endif ()

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

add_executable(inertia_calculator_node src/inertia_calculator_node.cpp)
target_link_libraries(inertia_calculator_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS inertia_calculator_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )