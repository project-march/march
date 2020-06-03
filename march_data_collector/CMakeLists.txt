cmake_minimum_required(VERSION 3.0.2)
project(march_data_collector)

find_package(catkin REQUIRED COMPONENTS
    control_msgs
    geometry_msgs
    march_shared_resources
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    visualization_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    march_shared_resources
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    visualization_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME} scripts/esp_adapter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
