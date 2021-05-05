# Copyright (C) 2021 Bas Volkers
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

"""Launch file for march_rqt_robot_monitor package."""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the march_rqt_robot_monitor, rqt_robot_monitor and diagnostic_aggregator nodes."""
    parameter_file = os.path.join(
        get_package_share_directory("march_rqt_robot_monitor"),
        "config",
        "analyzers.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="False",
                description="Whether to use simulation time.",
            ),
            DeclareLaunchArgument(
                name="rqt",
                default_value="true",
                description="Set to launch the rqt robot monitor.",
            ),
            Node(
                package="rqt_robot_monitor",
                executable="rqt_robot_monitor",
                namespace="rqt_robot_monitor",
                condition=IfCondition(LaunchConfiguration("rqt")),
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="diag_agg",
                parameters=[parameter_file],
            ),
            Node(
                package="march_rqt_robot_monitor",
                executable="march_rqt_robot_monitor_node",
                name="march_rqt_robot_monitor_node",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
