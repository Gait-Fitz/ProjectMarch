#[[
 Copyright (C) 2020 Bas Volkers, Olav de Haas
 Copyright (C) 2019 Cilia Claij, Rogier Krijnen

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
project(march_gain_scheduling)

find_package(catkin REQUIRED COMPONENTS
    march_shared_msgs
    std_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    march_shared_msgs
    std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test/one_step_linear_interpolation_test.py)
endif()
