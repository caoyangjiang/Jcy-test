cmake_minimum_required(VERSION 2.8)
project(CurlTest)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/CurlTest.cpp
)
# Library for handle http transactions
find_package( CURL REQUIRED )
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
 
include_directories(${CURL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES})
