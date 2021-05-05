#[[
 Copyright (C) 2020 Olav de Haas
 Copyright (C) 2019 Isha Dijks, Olav de Haas

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
project(march_temperature_sensor_controller)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    march_hardware_interface
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    controller_interface
    hardware_interface
    march_hardware_interface
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
)

include_directories(include SYSTEM ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/march_temperature_sensor_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES march_temperature_sensor_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

