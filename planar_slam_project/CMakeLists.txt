cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(planar_slam)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_slam main.cpp)
target_link_libraries (planar_slam ${PCL_LIBRARIES})
