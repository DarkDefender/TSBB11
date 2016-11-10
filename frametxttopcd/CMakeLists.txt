cmake_minimum_required(VERSION 2.8)
project(frametxt2pcd)

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} frametopcd.cc)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
