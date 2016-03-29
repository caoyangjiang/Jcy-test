cmake_minimum_required(VERSION 2.8)
project(ArithmeticCoder)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ArithmeticCoder.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
