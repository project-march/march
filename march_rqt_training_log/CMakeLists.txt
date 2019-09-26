cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_training_log)

find_package(catkin REQUIRED COMPONENTS
             roslint
             rospy
             rqt_gui
             rqt_gui_py
            )

catkin_package()
catkin_python_setup()

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )

install(DIRECTORY resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )

install(PROGRAMS scripts/rqt_launch_menu
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

roslint_python()
