# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2020 Thijs Raymakers
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

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "minimum_fake_temperature",
                default_value="10",
                description="Lower bound to generate fake temperatures from",
            ),
            DeclareLaunchArgument(
                "maximum_fake_temperature",
                default_value="30",
                description="Upper bound to generate fake temperatures from",
            ),
            Node(
                package="march_fake_sensor_data",
                executable="march_fake_sensor_data_node",
                name="fake_sensor_data",
                output="screen",
                parameters=[
                    {
                        "minimum_temperature": LaunchConfiguration(
                            "minimum_fake_temperature"
                        )
                    },
                    {
                        "maximum_temperature": LaunchConfiguration(
                            "maximum_fake_temperature"
                        )
                    },
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
