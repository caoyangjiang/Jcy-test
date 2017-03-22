cmake_minimum_required(VERSION 3.7)
project(PCLLoad)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/PCLLoad.cpp
${PROJECT_SOURCE_DIR}/src/PCLKernel.cu
${PROJECT_SOURCE_DIR}/src/PCLKernel.cuh
)

find_package( PCL REQUIRED )
find_package( OpenCV REQUIRED )
find_package( CUDA REQUIRED )
if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR CMAKE_COMPILER_IS_GNUCC
    OR CMAKE_COMPILER_IS_GNUCXX)
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  set(CUDA_NVCC_FLAGS "-std=c++11")
endif()
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_61,code=sm_61")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_52,code=sm_52")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=sm_50")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_35,code=sm_35")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30")

# for visual studio
string(REPLACE "/w44668" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "/sdl" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")


source_group("src" FILES ${PROJECT_SRCS})
 
cuda_add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${PCL_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC
                          $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
