# Copyright (C) 2021 Bas Volkers, Thijs Raymakers
# Copyright (C) 2020 Katja Schmahl
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

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """
    Launch file to launch rqt note taker.
    """

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Whether to use simulation time",
            ),
            Node(
                package="march_rqt_gait_version_tool",
                executable="gait_version_tool",
                output="screen",
                name="gait_version_tool",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
