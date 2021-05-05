# Copyright (C) 2021 Thijs Raymakers
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

import os
from glob import glob, iglob
from setuptools import setup

package_name = "march_gait_selection"


def data_files():
    """Generates the list of data files necessary for gait selection, the gait and subgait files
    required for testing are taken from the ros1 directory to decrease duplication."""
    test_gait_files_source = "test/testing_gait_files"
    data = [
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "test", "resources"),
            [os.path.join(test_gait_files_source, "default.yaml")],
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ]
    for file in iglob(
        os.path.join(test_gait_files_source, "**", "*.subgait"), recursive=True
    ):
        data.append(
            (
                os.path.join(
                    "share",
                    package_name,
                    "test",
                    "resources",
                    os.path.dirname(os.path.relpath(file, test_gait_files_source)),
                ),
                [file],
            )
        )
    for file in iglob(
        os.path.join(test_gait_files_source, "**", "*.gait"), recursive=True
    ):
        data.append(
            (
                os.path.join(
                    "share",
                    package_name,
                    "test",
                    "resources",
                    os.path.dirname(os.path.relpath(file, test_gait_files_source)),
                ),
                [file],
            )
        )
    return data


setup(
    name=package_name,
    version="0.0.0",
    packages=[
        package_name,
        "march_gait_selection.state_machine",
        "march_gait_selection.dynamic_gaits",
    ],
    data_files=data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project March",
    maintainer_email="software@projectmarch.nl",
    description="Package that is responsible for receiving input from the input device, "
    "determining whether a request is valid and executing the gait",
    license="TODO: License declaration",
    tests_require=["pytest", "unittest"],
    entry_points={
        "console_scripts": [
            "march_gait_selection = march_gait_selection.gait_selection_node:main"
        ],
    },
)
