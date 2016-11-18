cmake_minimum_required(VERSION 2.8)
project(frametxt2pcd)

if(APPLE)
	link_directories(/usr/local/opt/opencv3/share/OpenCV/3rdparty/lib/ /usr/local/opt/opencv3/lib)
endif()

include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} frametopcd.cc)
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES})
