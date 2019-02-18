cmake_minimum_required(VERSION 2.8.12)

project(fixedWheelKrang)

find_package(DART 6.3.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "Main.cpp" "MyWindow.cpp" "MyWindow.hpp" "Controller.cpp" "Controller.hpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} nlopt)
