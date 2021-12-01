cmake_minimum_required(VERSION 3.5)
project(march_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    SYSTEM
)

include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS})

ament_package()
