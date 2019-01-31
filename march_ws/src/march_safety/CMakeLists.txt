cmake_minimum_required(VERSION 2.8.3)
project(march_safety)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        march_shared_resources
        roscpp
        roslint
        sensor_msgs
        std_msgs
        )

catkin_package(
        CATKIN_DEPENDS
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})
include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(march_safety_node src/SafetyNode.cpp)
add_dependencies(march_safety_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(march_safety_node ${catkin_LIBRARIES})