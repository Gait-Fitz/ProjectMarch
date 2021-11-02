cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(foot_position_finder)

find_package(PCL 1.7.2 REQUIRED)
find_package(CGAL)
find_package(realsense2 REQUIRED)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES CONFIGURE_DEPENDS
        "src/*.cpp"
        "src/*/*.cpp"
)

add_executable (${PROJECT_NAME} ${SOURCES})
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})




set(default_build_type "Release")
