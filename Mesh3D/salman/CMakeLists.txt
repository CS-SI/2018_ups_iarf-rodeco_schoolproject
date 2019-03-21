# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.
cmake_minimum_required(VERSION 3.1)
project(Salman)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

# setup packages

find_package(CGAL REQUIRED COMPONENTS Qt5 Core)
find_package(OpenCV REQUIRED)
find_package(netCDF REQUIRED)
add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)

#create the executable of the application
add_executable(Triangulation triangulation.cpp)

#link it with the required CGAL libraries
target_link_libraries(Triangulation CGAL::CGAL CGAL::CGAL_Qt5 CGAL::CGAL_Core)
target_link_libraries(Triangulation ${OpenCV_LIBS})
target_link_libraries(Triangulation ${netCDF_LIBRARIES})







