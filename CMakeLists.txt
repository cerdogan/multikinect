cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_grabber)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -std=c++0x -g)

add_executable (openni_grabber openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})
add_executable (all4 all4.cpp)
target_link_libraries (all4 ${PCL_LIBRARIES})
add_executable (visualizer visualizer.cpp)
target_link_libraries (visualizer ${PCL_LIBRARIES})

