cmake_minimum_required(VERSION 3.0.2)
project(march_gain_scheduling)

find_package(catkin REQUIRED COMPONENTS
    march_shared_resources
    std_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    march_shared_resources
    std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test/one_step_linear_interpolation_test.py)
endif()
