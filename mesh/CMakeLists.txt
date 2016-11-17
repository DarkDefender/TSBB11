cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(concaveHull)

list( APPEND CMAKE_CXX_FLAGS "-O3 -march=native ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (concaveHull poisson.cpp)
target_link_libraries (concaveHull ${PCL_LIBRARIES} popt)