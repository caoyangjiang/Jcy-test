cmake_minimum_required(VERSION 2.8)
project(ByteRemap)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ByteRemap.cpp
)
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
 
