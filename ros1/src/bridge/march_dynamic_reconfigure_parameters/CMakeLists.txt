cmake_minimum_required(VERSION 3.0.2)
project(march_dynamic_reconfigure_parameters)

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
