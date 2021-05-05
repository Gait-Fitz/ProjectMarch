#[[
 Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 Copyright (C) 2020 Bas Volkers

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
project(march_safety)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(march_utility REQUIRED)

add_library(${PROJECT_NAME}
        src/input_device_safety.cpp
        src/safety_handler.cpp
        src/temperature_safety.cpp
)
ament_target_dependencies(${PROJECT_NAME} march_shared_msgs rclcpp sensor_msgs std_msgs urdf march_utility)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# the executable that is run in ROS and links with the library defined above
add_executable(${PROJECT_NAME}_node src/safety_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node march_shared_msgs rclcpp sensor_msgs std_msgs urdf march_utility)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/)

#if(BUILD_TESTING)
#    find_package(ros_testing REQUIRED)
#    install(DIRECTORY test
#            DESTINATION share/${PROJECT_NAME}/)
#    ament_python_install_package(test)
#    add_ros_test(test/launch/march_safety_connection.launch.py)
#    add_ros_test(test/launch/march_safety_temperature.launch.py)
#endif()

ament_package()
