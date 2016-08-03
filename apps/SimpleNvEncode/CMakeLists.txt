cmake_minimum_required(VERSION 2.8)
project(SimpleNvEncode)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/NvEncode.cpp
)

find_package( CUDA REQUIRED )

include_directories(${CUDA_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} nvidia-encode)
target_link_libraries(${PROJECT_NAME} cuda)
