cmake_minimum_required(VERSION 2.8)
project(BitSet)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/BitSet.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
