cmake_minimum_required(VERSION 2.8)
project(modules)

add_subdirectory(Boost)
add_subdirectory(OpenCV)
add_subdirectory(Pcl)
add_subdirectory(Eigen)
add_subdirectory(ObjFile)

if(NOT APPLE)
add_subdirectory(Cuda)
endif()