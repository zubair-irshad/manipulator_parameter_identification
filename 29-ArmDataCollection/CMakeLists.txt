cmake_minimum_required(VERSION 2.8)
project(forwardKinematics)

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
set(CMAKE_CXX_FLAGS "-g -std=gnu++0x")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
#link_libraries(protobuf-c amino ach kore somatic filter stdc++ ncurses blas lapack)
link_libraries(protobuf-c amino ach somatic stdc++ blas lapack)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Dart and Eigen
find_package(Eigen3 REQUIRED)
find_package(DART 6 REQUIRED COMPONENTS utils-urdf)
include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${DART_INCLUDE_DIRS})

# Include the project files and the common helpers for all the experiments
include_directories(../experiments/common)

# Collect the source, script and fortran files
file(GLOB scripts_source "*.cpp")

#file(GLOB scripts_source "exe/00-singlearm.cpp")
LIST(SORT scripts_source)

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
	get_filename_component(script_base ${script_src_file} NAME_WE)
	message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
	add_executable(${script_base} ${script_src_file})
	# target_link_libraries(${script_base} Main) 
	target_link_libraries(${script_base} ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 
	add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)
message(STATUS " ")
