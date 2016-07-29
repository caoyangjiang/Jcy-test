cmake_minimum_required(VERSION 2.8)
project(MoofParser)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/MoofParser.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
