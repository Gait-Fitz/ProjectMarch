cmake_minimum_required(VERSION 3.0.2)
project(march_system_identification)

find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        std_msgs
        )

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        sensor_msgs
        std_msgs
)

catkin_install_python(PROGRAMS 
        src/${PROJECT_NAME}/frequency_response_analysis
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
