cmake_minimum_required(VERSION 2.8.3)
project(march_launch)

find_package(catkin REQUIRED COMPONENTS
    march_description
    march_simulation
    march_state_machine
    roslint
    rosserial_python
    rostest
    rqt_gui
    rqt_gui_py
)

catkin_python_setup()
catkin_package()

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch resource rqt_config rqt_multiplot 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/march_git_branch_check scripts/march_rqt_software_check
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB_RECURSE lintfiles "src/*.py")
roslint_python(${lintfiles})

if (CATKIN_ENABLE_TESTING)
endif ()
