cmake_minimum_required(VERSION 2.8)
project(frametxt2pcd)

if(APPLE)
	link_directories(/usr/local/opt/opencv3/share/OpenCV/3rdparty/lib/ /usr/local/opt/opencv3/lib)
endif()

include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv)
pkg_search_module(EIGEN3 REQUIRED eigen3)

include_directories(${OPENCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_executable(${PROJECT_NAME} frametopcd.cc)

add_executable(orbsetting calibtoorb.cc)
target_link_libraries(orbsetting ${OPENCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES})

add_executable(rotcam rotcam.cc)
target_link_libraries(rotcam ${EIGEN3_LIBRARIES})
