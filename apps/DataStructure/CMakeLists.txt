cmake_minimum_required(VERSION 2.8)
project(Data)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Data.cpp
)

find_package( Boost REQUIRED )
include_directories(${Boost_INCLUDE_DIR})
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
 
