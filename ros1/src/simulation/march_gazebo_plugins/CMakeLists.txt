#[[
 Copyright (C) 2021 Katja Schmahl, Thijs Raymakers
 Copyright (C) 2020 Bas Volkers, Katja Schmahl, Olav de Haas,
                    Roel Vos, Thijs Raymakers

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
project(march_gazebo_plugins LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    march_shared_msgs
    roscpp
)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
)
include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} contact
    CATKIN_DEPENDS march_shared_msgs roscpp
)

add_library(${PROJECT_NAME}
    src/com_controller_plugin.cpp
    src/obstacle_controller.cpp
    src/walk_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS}/${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} yaml-cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Werror)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#####
# Add contact plugin
#####
add_library(contact
    src/contact_plugin.cpp
)

target_include_directories(contact PRIVATE ${catkin_INCLUDE_DIRS}/contact)
target_link_libraries(contact PRIVATE ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_compile_features(contact PRIVATE cxx_std_17)
target_compile_options(contact PRIVATE -Wall -Wextra -Werror)

install(DIRECTORY include/contact/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/contact
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS contact
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
