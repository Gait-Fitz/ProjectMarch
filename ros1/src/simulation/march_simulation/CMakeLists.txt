#[[
 Copyright (C) 2021 Katja Schmahl
 Copyright (C) 2020 Olav de Haas, Roel Vos
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

cmake_minimum_required(VERSION 3.10.2)
project(march_simulation)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/upload_joint_names scripts/set_obstacle_dimensions
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch obstacles worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
