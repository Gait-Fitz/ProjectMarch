#[[
 Copyright (C) 2020 Bas Volkers, Katja Schmahl

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

cmake_minimum_required(VERSION 3.5)
project(march_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

include(cmake/${PROJECT_NAME}-extras.cmake)

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
        )

set(XACRO_FILES
    urdf/march3.xacro
    urdf/march4.xacro
    urdf/march4_with_koen_lengths.xacro
    urdf/test_joint_linear.xacro
    urdf/test_joint_rotational.xacro
)

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)

build_and_install_xacro_files(${XACRO_FILES})
