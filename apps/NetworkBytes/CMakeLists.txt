cmake_minimum_required(VERSION 3.5)
project(NetworkBytes)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/NetworkBytes.cpp
)
# Library for parsing Dash media presentation description

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

