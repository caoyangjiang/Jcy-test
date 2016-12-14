cmake_minimum_required(VERSION 3.5)
project(BitOp)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/BitOp.cpp
)

# Find dependent libraries
find_package( Boost REQUIRED )

# Include paths
include_directories(${Boost_INCLUDE_DIR})

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})