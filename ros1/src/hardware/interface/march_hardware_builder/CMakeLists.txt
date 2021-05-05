#[[
 Copyright (C) 2021 Bas Volkers
 Copyright (C) 2020 Olav de Haas, Roel Vos, Rutger van Beek
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
project(march_hardware_builder)

add_compile_options(-std=c++17 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    march_hardware
    roscpp
    roslib
    urdf
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    march_hardware
    roscpp
    roslib
    urdf
    LIBRARIES ${PROJECT_NAME}
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    append_coverage_compiler_flags()
endif()

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/hardware_builder.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY robots
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_test
        test/absolute_encoder_builder_test.cpp
        test/allowed_robot_test.cpp
        test/imc_builder_test.cpp
        test/motor_controller_builder_test.cpp
        test/incremental_encoder_builder_test.cpp
        test/joint_builder_test.cpp
        test/pdb_builder_test.cpp
        test/test_runner.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME})

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*")
        add_code_coverage(
            NAME coverage_report
            DEPENDENCIES ${PROJECT_NAME}_test
        )
    endif()
endif()
