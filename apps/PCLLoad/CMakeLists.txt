cmake_minimum_required(VERSION 3.5)
project(PCLLoad)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/PCLLoad.cpp
${PROJECT_SOURCE_DIR}/src/PCLKernel.cu
${PROJECT_SOURCE_DIR}/src/PCLKernel.cuh
)

# Find dependent libraries
find_package( PCL REQUIRED )
find_package( OpenCV REQUIRED )
find_package( CUDA REQUIRED )
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_52,code=sm_52;")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=sm_50;")

# Include paths
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
include_directories(../../modules/PCL/include)

if(MSVC)
  string(REGEX REPLACE "/W[0-4]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  list(APPEND CUDA_NVCC_FLAGS "-std=c++11;")
endif()

# Compile project into application
cuda_add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

# Link application to libraries
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})