cmake_minimum_required(VERSION 3.5)
project(CurlTest)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/CurlTest.cpp
)

# Find dependent libraries
find_package( CURL REQUIRED )

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

# Include paths
include_directories(${CURL_INCLUDE_DIRS})

# Link application to libraries
target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES})
