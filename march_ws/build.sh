#!/usr/bin/env bash

catkin_make || exit 1

# cpplinter.
catkin_make roslint_march_main || exit 1

# Catkin lint, fail on errors only. TODO remove missing_directory.
catkin_lint src/*/ --ignore missing_directory --ignore literal_project_name --explain -W0 || exit 1

# Run the tests, ensuring the path is set correctly.
source devel/setup.bash || exit 1
catkin_make run_tests && catkin_test_results || exit 1


