cmake_minimum_required(VERSION 2.8)
project(SimpleMatrix)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/SimpleMatrix.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
