cmake_minimum_required(VERSION 2.8)
project(NetworkBytes)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/NetworkBytes.cpp
)
# Library for parsing Dash media presentation description

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

