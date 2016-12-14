cmake_minimum_required(VERSION 3.5)
project(jcy_cuda)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/CUDATest.cu
${PROJECT_SOURCE_DIR}/src/CuRotator.cu
${PROJECT_SOURCE_DIR}/src/CudaColorSpaceCVT.cu
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/Cuda/CUDAGlobal.cuh
${PROJECT_SOURCE_DIR}/include/Jcy/Cuda/CUDATest.cuh
${PROJECT_SOURCE_DIR}/include/Jcy/Cuda/CuRotator.cuh
${PROJECT_SOURCE_DIR}/include/Jcy/Cuda/CudaColorSpaceCVT.h
${PROJECT_SOURCE_DIR}/include/Jcy/Cuda/CudaConfig.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

# Find dependent libraries
find_package( CUDA REQUIRED )

# Add extra flags to NVCC
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_52,code=sm_52;")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=sm_50;")
if(MSVC)
  string(REGEX REPLACE "/W[0-4]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  list(APPEND CUDA_NVCC_FLAGS "-std=c++11;")
endif()

# Include paths
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})

# Link module to external libraries
cuda_add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")
