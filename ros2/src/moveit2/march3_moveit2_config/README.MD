# MoveIt2 demo package for march 3 urdf in ROS2

This package is a port of the MoveIt Setup Assistant generated package of the march 3 urdf. 

## How to create:

1. Generate demo package using MoveIt Setup Assistant in MoveIt1 (using Noetic)

2. Change CMakeLists.txt:
    * Set cmake_minimum_required to 3.3
    * Change `find_package(catkin REQUIRED)` to `find_package(ament_cmake REQUIRED)`
    * Change `catkin_package()` to `ament_package()`
    * Change install to `install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})`
3. Change Package.xml:
    * Set package format to 2
    * Change catkin to ament_cmake
    * Change run_depend to exec_depend
    * Add amen_cmake as export
4. Create a `demo.launch.py` file based on: https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/move_group_interface/launch/move_group.launch.py

Step 2 till 4 can be found in commit:
https://gitlab.com/project-march/march/-/commit/15d6a08a2f5c4575db66bbdf8d4015d1dd12539f