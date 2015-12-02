cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_grabber)

find_package(PCL 1.2 REQUIRED)

# Include the OpenCV library
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

# Include the PCL library
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -std=c++0x -g)

# Collect the source, script and test files
file(GLOB scripts_source "exe/*.cpp")
LIST(SORT scripts_source)

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
    get_filename_component(script_base ${script_src_file} NAME_WE)
    message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
    add_executable(${script_base} ${script_src_file})
		target_link_libraries (${script_base} ${PCL_LIBRARIES})
    add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)
message(STATUS " ")
