cmake_minimum_required(VERSION 2.8)
project(EigenTest)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/main.cpp
${PROJECT_SOURCE_DIR}/src/EigenTest.cpp
${PROJECT_SOURCE_DIR}/inc/EigenTest.h
)

find_package( Eigen3 REQUIRED )

include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories(../../modules/Eigen/include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} jcy_eigen)
target_link_libraries(${PROJECT_NAME} gtest_main)

