cmake_minimum_required(VERSION 3.15)

project(Stewart VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(Stewart main.cxx)

target_link_libraries(Stewart ${ARMADILLO_LIBRARIES})
