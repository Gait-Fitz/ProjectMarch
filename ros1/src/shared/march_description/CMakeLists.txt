#[[
 Copyright (C) 2020 Bas Volkers, Olav de Haas
 Copyright (C) 2019 Olav de Haas, Tim Buckers

 This program is free software: you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 Version 3 as published by the Free Software Foundation WITH
 additional terms published by Project MARCH per section 7 of
 the GNU General Public License Version 3.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License INCLUDING the additional terms for
 more details.

 You should have received a copy of the GNU General Public License
 AND the additional terms along with this program. If not,
 see <https://projectmarch.nl/s/LICENSE> and
 <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.
]]

cmake_minimum_required(VERSION 3.0.2)
project(march_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

include(cmake/${PROJECT_NAME}-extras.cmake)

set(XACRO_FILES
    "../../../../ros2/src/shared/march_description/urdf/march3.xacro"
    "../../../../ros2/src/shared/march_description/urdf/march4.xacro"
    "../../../../ros2/src/shared/march_description/urdf/march4_with_koen_lengths.xacro"
    "../../../../ros2/src/shared/march_description/urdf/test_joint_rotational.xacro"
    "../../../../ros2/src/shared/march_description/urdf/test_joint_linear.xacro"
)
build_and_install_xacro_files(${XACRO_FILES})

install(DIRECTORY "../../../../ros2/src/shared/march_description/urdf"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
