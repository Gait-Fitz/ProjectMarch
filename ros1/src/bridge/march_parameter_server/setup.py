# Copyright (C) 2021 Thijs Raymakers
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

# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
import os

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=["march_parameter_server"],
    package_dir={"": "src"},
    scripts=[os.path.join("scripts", "march_parameter_server_node")],
)

setup(**setup_args)
