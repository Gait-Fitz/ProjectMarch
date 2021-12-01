cmake_minimum_required(VERSION 3.5)
project(march_simulation)

find_package(ament_cmake REQUIRED)

find_package(controller_manager REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joint_state_controller REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(march_description REQUIRED)
find_package(march_gazebo_plugins REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rospy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdfdom_py REQUIRED)
find_package(xacro REQUIRED)

ament_package()

install(PROGRAMS scripts/set_obstacle_dimensions scripts/republish_pointclouds.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch obstacles worlds
    DESTINATION share/${PROJECT_NAME}
)