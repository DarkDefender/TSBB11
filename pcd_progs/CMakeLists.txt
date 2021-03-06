cmake_minimum_required (VERSION 2.6)
project (cutting2)

if(APPLE)
	link_directories(/usr/local/opt/opencv3/share/OpenCV/3rdparty/lib/ /usr/local/opt/opencv3/lib)
endif()

list( APPEND CMAKE_CXX_FLAGS "-Wall -std=c++14 -O2 -march=native ${CMAKE_CXX_FLAGS}")

#add_subdirectory(libs)
add_executable(${PROJECT_NAME} cutting2.cpp)
add_executable(merge normal_calc_and_merge.cpp)
add_executable(cleanup clean_up.cpp)

include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv)
find_package(PCL 1.2 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(SYSTEM
	${OPENCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
	${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cleanup
	${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(merge
	${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
