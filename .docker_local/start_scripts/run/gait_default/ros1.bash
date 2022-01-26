#!/bin/bash

source /opt/ros/noetic/setup.bash
source "${HOME}"/march/ros1/install/local_setup.bash
cd "${HOME}"/march/ros1/ || exit
echo "Launching ros1 with ROS_ARGS: ${ROS_ARGS} and ROS1_ARGS: ${ROS1_ARGS}"
exec roslaunch march_launch march.launch if_name:=enp2s0f0 ${GAIT_TYPE_ARGS} ${ROS_ARGS} ${ROS1_ARGS}
