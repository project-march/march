cmake_minimum_required(VERSION 2.8.3)
project(march_moveit)

find_package(catkin REQUIRED)
catkin_python_setup()

catkin_package()
catkin_install_python(PROGRAMS scripts/move_group.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
