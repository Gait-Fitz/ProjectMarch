cmake_minimum_required(VERSION 3.5)
project(march_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

include_directories(SYSTEM)
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS})
include_directories(march_shared_msgs)

message(ERROR "foo include dir: ${PROJECT_NAME}")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_libraries(contact)
ament_export_dependencies(march_shared_msgs)
ament_export_dependencies(rclcpp)
ament_package()

# add_library(${PROJECT_NAME}
#     src/com_controller_plugin.cpp
#     src/obstacle_controller.cpp
#     src/walk_controller.cpp
# )

add_library(com_controller_plugin src/com_controller_plugin.cpp)
ament_target_dependencies(com_controller_plugin march_shared_msgs rclcpp)