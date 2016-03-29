cmake_minimum_required(VERSION 2.8)
project(AvxTest)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/AvxTest.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

