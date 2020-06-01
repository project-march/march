cmake_minimum_required(VERSION 3.0.2)
project(march_moveit)

find_package(catkin REQUIRED COMPONENTS
    march_description
    )

catkin_package(CATKIN_DEPENDS
    march_description
    )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
