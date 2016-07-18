cmake_minimum_required(VERSION 2.8)
project(ObjToTriangle)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ObjToTriangle.cpp
)

find_package( PCL REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})


