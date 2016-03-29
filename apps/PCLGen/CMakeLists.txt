cmake_minimum_required(VERSION 2.8)
project(PCLGen)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/PCLGen.cpp
)

find_package( PCL REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
include_directories(../../modules/PCL/include)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

#target_link_libraries(${PROJECT_NAME} jcy_eigen)

