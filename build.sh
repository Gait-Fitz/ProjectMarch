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

# Build entire workspace
catkin build --summarize  --no-notify|| build_failed "Could not build workspace"
source devel/setup.bash

# Catkin lint
catkin lint -W2 --pkg march_fake_sensor_data || build_failed "Catkin lint failed in march_fake_sensor_data"
catkin lint -W2 --pkg march_simulation || build_failed "Catkin lint failed in march_simulation"

# Roslint, skip the march_simulation package as it does not and should not contain any code.
catkin build --no-deps --verbose march_fake_sensor_data --no-notify --catkin-make-args roslint || build_failed "Roslint failed in march_fake_sensor_data"

# Run all tests in the workspace, including roslaunch-checks if they exist
catkin build --summarize --catkin-make-args run_tests && catkin_test_results build/ --verbose || build_failed "Tests failed"
