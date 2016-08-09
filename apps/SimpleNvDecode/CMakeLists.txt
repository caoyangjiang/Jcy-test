cmake_minimum_required(VERSION 2.8)
project(SimpleNvDecode)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/NvDecode.cpp
)

find_package( CUDA REQUIRED )

include_directories(${CUDA_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} nvcuvid)
target_link_libraries(${PROJECT_NAME} cuda)
