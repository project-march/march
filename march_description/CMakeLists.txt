cmake_minimum_required(VERSION 3.0.2)
project(march_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

include(cmake/${PROJECT_NAME}-extras.cmake)

set(XACRO_FILES
    urdf/march3.xacro
    urdf/march4.xacro
    urdf/test_joint_linear.xacro
    urdf/test_joint_rotational.xacro
)
build_and_install_xacro_files(${XACRO_FILES})

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
