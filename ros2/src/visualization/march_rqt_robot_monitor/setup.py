# Copyright (C) 2021 Bas Volkers, Thijs Raymakers
# Copyright (C) 2020 Bas Volkers, Joris Weeda
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

"""Setup file for the march_rqt_robot_monitor package."""

from setuptools import setup
from glob import glob
import os

package_name = "march_rqt_robot_monitor"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, f"{package_name}.diagnostic_analyzers"],
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="An RQT plugin for monitoring log messages and writing logs.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"{package_name}_node = {package_name}.updater:main"],
    },
)
