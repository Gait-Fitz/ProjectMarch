cmake_minimum_required(VERSION 3.0.2)
project(march_system_identification)

find_package(catkin REQUIRED COMPONENTS
        control_msgs
        march_shared_msgs
        std_msgs
        )

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        control_msgs
        march_shared_msgs
        std_msgs
)

catkin_install_python(PROGRAMS 
        nodes/frequency_response_analysis
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
