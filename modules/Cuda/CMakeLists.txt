cmake_minimum_required(VERSION 2.8)
project(jcy_cuda)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/CUDATest.cu
${PROJECT_SOURCE_DIR}/src/CuRotator.cu
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/Cuda/CUDAGlobal.cuh
${PROJECT_SOURCE_DIR}/include/Jcy/Cuda/CUDATest.cuh
${PROJECT_SOURCE_DIR}/include/Jcy/Cuda/CuRotator.cuh
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

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

include_directories(${PROJECT_SOURCE_DIR}/include)
cuda_add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")