cmake_minimum_required(VERSION 2.8)
project(CUDATestExample)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/CUDATestExample.cpp
)

include_directories(../../modules/CUDATest/inc)

find_package( CUDA REQUIRED )
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_52,code=sm_52;")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=sm_50;")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_35,code=sm_35;")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30;")
if(MSVC)
  string(REGEX REPLACE "/W[0-4]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  list(APPEND CUDA_NVCC_FLAGS "-std=c++11;")
endif()

cuda_add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} CUDATest)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
