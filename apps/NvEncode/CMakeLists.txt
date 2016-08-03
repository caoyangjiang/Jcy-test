cmake_minimum_required(VERSION 2.8)
project(NvEncode)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/NvEncoder.cpp
${PROJECT_SOURCE_DIR}/src/dynlink_cuda.cpp
${PROJECT_SOURCE_DIR}/src/NvHWEncoder.cpp
${PROJECT_SOURCE_DIR}/src/NvEncoder.h
${PROJECT_SOURCE_DIR}/src/NvHWEncoder.h
${PROJECT_SOURCE_DIR}/src/dynlink_cuda.h
${PROJECT_SOURCE_DIR}/src/dynlink_cuda_cuda.h
${PROJECT_SOURCE_DIR}/src/nvCPUOPSys.h
${PROJECT_SOURCE_DIR}/src/nvEncodeAPI.h
${PROJECT_SOURCE_DIR}/src/nvFileIO.h
${PROJECT_SOURCE_DIR}/src/nvUtils.h
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
target_link_libraries(${PROJECT_NAME} nvidia-encode dl)
