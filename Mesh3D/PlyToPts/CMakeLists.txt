cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
project(Ply_to_pts)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pts_writer pts_writer.cpp)
target_link_libraries (pts_writer ${PCL_LIBRARIES} )
