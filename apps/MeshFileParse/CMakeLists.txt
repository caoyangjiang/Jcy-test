cmake_minimum_required(VERSION 2.8)
project(MeshFileParse)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/MeshFileParse.cpp
)

include_directories(../../modules/ObjFile/include)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
target_link_libraries(${PROJECT_NAME} jcy_objfile)
