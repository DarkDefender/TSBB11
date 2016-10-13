cmake_minimum_required (VERSION 2.6)
project (opencv_test)

if(APPLE)
	link_directories(/usr/local/opt/opencv3/share/OpenCV/3rdparty/lib/ /usr/local/opt/opencv3/lib)
endif()

list( APPEND CMAKE_CXX_FLAGS "-Wall -ggdb -std=c++14 ${CMAKE_CXX_FLAGS}")

#add_subdirectory(libs)
add_executable(${PROJECT_NAME} main.cpp camParser.cpp)

include(FindPkgConfig)

pkg_search_module(OPENCV REQUIRED opencv)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(SYSTEM
	${OPENCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
	${OPENCV_LIBRARIES})
