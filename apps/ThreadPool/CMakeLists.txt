cmake_minimum_required(VERSION 2.8)
project(ThreadPool)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ThreadPool.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
 
