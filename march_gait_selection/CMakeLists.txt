cmake_minimum_required(VERSION 3.0.2)
project(march_gait_selection)

find_package(catkin REQUIRED COMPONENTS
    march_shared_resources
    std_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
    march_shared_resources
    std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/launch/testing_gait_selection.test)
    add_rostest(test/launch/testing_dynamic_gaits.test)
endif()
