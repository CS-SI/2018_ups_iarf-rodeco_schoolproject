cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(PCL_RECONSTRUCTION_TEST)

# Project repertory

SET(CMAKE_CURRENT_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Find packages

find_package(PCL 1.7 REQUIRED)

# include appropriate directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Source file of project
SET(EXECUTABLES greedyTriangulation poissonReconstruction organizedFastMesh marchingCubesRBF)

# Create libraries and executables

foreach(_name ${EXECUTABLES})
  add_executable(${_name} ${_name}.cpp)
  target_link_libraries(${_name} ${PCL_LIBRARIES})
endforeach()
