cmake_minimum_required(VERSION 2.8.3)
project(march_safety)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        march_shared_resources
        roscpp
        roslint
        rostest
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

set(sourcefiles
        src/SafetyNode.cpp
        src/TemperatureSafety.h
        src/TemperatureSafety.cpp
        )

add_library(march_safety ${sourcefiles})

add_executable(march_safety_node src/SafetyNode.cpp src/TemperatureSafety.cpp src/TemperatureSafety.h)
add_dependencies(march_safety_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(march_safety_node ${catkin_LIBRARIES})



#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(march_safety_test
            test/error.test
            test/TestError.cpp
            )
    target_link_libraries(march_safety_test ${catkin_LIBRARIES} march_safety gtest gmock)

endif ()

