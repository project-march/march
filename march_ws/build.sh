#!/usr/bin/env bash

catkin_make || exit 1

# cpplinter.
catkin_make roslint_march_main || exit 1

# Catkin lint, fail on errors only. TODO remove missing_directory.
catkin_lint src/*/ --ignore missing_directory --ignore literal_project_name --ignore missing_install_target --explain -W1 || exit 1

# Run the tests, ensuring the path is set correctly.
source devel/setup.bash || exit 1
catkin_make run_tests && catkin_test_results || exit 1

for directory in src/*/launch
do
    rosrun roslaunch roslaunch-check $directory || exit 1
done

if [ -x "$(command -v notify-send)" ]; then
    notify-send -i $PWD/walking.png "Your local build has passed!"
fi
