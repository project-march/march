#!/usr/bin/env bash

build_passed () {
    notify "Your local build has passed!"
}


build_failed () {
    notify "$1"
    exit 1
}

notify () {
    if [[ -x "$(command -v notify-send)" ]]; then
        notify-send -i $PWD/walking.png "$1"
    fi
}
skip_package() {
    declare -a excluded_packages=("march_moveit" "march_description" "march_launch" "march_gazebo" "march_rqt_launch_menu" "march_rqt_input_device")
    for excluded_package in "${excluded_packages[@]}"
    do
        if [[ "$1" == ${excluded_package} ]]; then
            echo 1
            return 1
        fi
    done
    echo 0
}

#catkin_make --pkg march_shared_resources || build_failed "march shared resources catkin_make failed"
#
#catkin_make || build_failed "Catkin_make failed"
#
#for directory in $(find -O3 -L src/ -name "CMakeLists.txt")
#do
#
#    if [[ "$directory" == *"march_"* ]]
#    then
#            package_name=$(basename $(dirname "${directory}"))
#            package_roslint="roslint_$package_name"
#            should_skip="$(skip_package ${package_name})"
#            echo ${should_skip}
#            if [[ ${should_skip} == "0" ]]
#            then
#                catkin_make ${package_roslint} || build_failed "ros_lint failed in $package_name"
#            fi
#    fi
#done
#
#
#
## Catkin lint, fail on errors only. TODO remove missing_directory.
#catkin_lint src/*/ --ignore missing_directory --ignore literal_project_name --ignore missing_install_target --explain -W2 || build_failed "catkin_lint failed"

# Pycodestyle runs on the state_machine python package

for directory in $(find -O3 -L src/ -name "setup.py")
do
    if [[ "$directory" == *"march_"* ]]
    then
            package_path=$(dirname "${directory}")
            echo ${package_path}
            pycodestyle ${package_path} || build_failed "pycodestyle failed in ${package_path}"
    fi
done



# Run the tests, ensuring the path is set correctly.
#source devel/setup.bash || exit 1
#catkin_make run_tests && catkin_test_results || build_failed "Tests failed"

#for directory in $(find -O3 -L src/ -type d -name "launch")
#do
#    if [[ "$directory" != *"march_moveit"* ]]
#    then
#        rosrun roslaunch roslaunch-check $directory || build_failed "roslaunch-check failed in directory $directory"
#    fi
#done


build_passed
