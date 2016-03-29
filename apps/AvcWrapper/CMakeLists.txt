cmake_minimum_required(VERSION 2.8)
project(AvcWrapper)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/AvcWrapper.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
