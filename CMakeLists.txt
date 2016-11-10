cmake_minimum_required (VERSION 2.6)
project (cutting2)

if(APPLE)
	link_directories(/usr/local/opt/opencv3/share/OpenCV/3rdparty/lib/ /usr/local/opt/opencv3/lib)
endif()

list( APPEND CMAKE_CXX_FLAGS "-Wall -ggdb -std=c++14 ${CMAKE_CXX_FLAGS}")

#add_subdirectory(libs)
add_executable(${PROJECT_NAME} cutting2.cpp)

include(FindPkgConfig)

pkg_search_module(OPENCV REQUIRED opencv)
find_package(PCL 1.2 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(SYSTEM
	${OPENCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
	${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
