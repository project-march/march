cmake_minimum_required(VERSION 2.8.3)
project(march_sound_scheduler)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
    code_coverage
    march_shared_resources
    roscpp
    roslint
    rostest
    sound_play
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
)

set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
    "src/*.cpp"
    "src/*.h"
)

roslint_cpp(${lintfiles})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    include/march_sound_scheduler/ScheduledSound.h
    include/march_sound_scheduler/Scheduler.h
    src/ScheduledSound.cpp
    src/Scheduler.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/SoundSchedulerNode.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
