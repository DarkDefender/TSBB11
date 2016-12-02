cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(texturemapping)
 
if(APPLE)
	link_directories(/usr/local/opt/opencv3/share/OpenCV/3rdparty/lib/ /usr/local/opt/opencv3/lib)
endif()   
list( APPEND CMAKE_CXX_FLAGS "-O2 -march=native ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)
 
include(FindPkgConfig) 
pkg_search_module(OPENCV REQUIRED opencv)  
include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(texturemapping texturemapping.cc)
target_link_libraries(texturemapping ${PCL_LIBRARIES} ${OPENCV_LIBRARIES})
