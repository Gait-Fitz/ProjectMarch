# Copyright (C) 2021 Bas Volkers
# Copyright (C) 2020 Bas Volkers
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# Version 3 as published by the Free Software Foundation WITH
# additional terms published by Project MARCH per section 7 of
# the GNU General Public License Version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License INCLUDING the additional terms for
# more details.
#
# You should have received a copy of the GNU General Public License
# AND the additional terms along with this program. If not,
# see <https://projectmarch.nl/s/LICENSE> and
# <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="config_path",
                default_value=os.path.join(
                    get_package_share_directory("march_safety"),
                    "config",
                    "safety_settings.yaml",
                ),
                description="Path to the configuration for the safety settings",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
                description="Whether to use simulation time",
            ),
            Node(
                package="march_safety",
                executable="march_safety_node",
                name="safety_node",
                namespace="march",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_path"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                # If this node exits, the entire system is shutdown
                # This is the ROS2 equivalent of required:=true
                # See: https://ubuntu.com/blog/ros2-launch-required-nodes
                on_exit=Shutdown(),
            ),
        ]
    )
