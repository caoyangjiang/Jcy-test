cmake_minimum_required(VERSION 3.5)
project(SimpleNvEncode)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/NvEncode.cpp
)

find_package( CUDA REQUIRED )
find_package( NVCODEC REQUIRED )
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
include_directories(SYSTEM ${NVCODEC_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} nvidia-encode)
target_link_libraries(${PROJECT_NAME} cuda)
