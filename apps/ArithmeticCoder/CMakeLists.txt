cmake_minimum_required(VERSION 3.5)
project(ArithmeticCoder)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

find_package( SIMD REQUIRED )

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ArithmeticCoder.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
