cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_gait_selection)

find_package(catkin REQUIRED COMPONENTS
    roslint
    rospy
    rqt_gui
    rqt_gui_py
)

catkin_python_setup()
catkin_package()

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS scripts/rqt_gait_selection
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB_RECURSE lintfiles "src/*.py")
roslint_python(${lintfiles})
