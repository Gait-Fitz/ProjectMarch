cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(point_cloud)

find_package(PCL 1.7.2 REQUIRED)
find_package(CGAL)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable (point_cloud point_cloud.cpp)
add_executable (point_cloud CGAL.cpp)
target_link_libraries (point_cloud ${PCL_LIBRARIES})
target_link_libraries(point_cloud CGAL::CGAL)


