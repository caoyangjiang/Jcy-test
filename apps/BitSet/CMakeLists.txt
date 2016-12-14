cmake_minimum_required(VERSION 3.5)
project(BitSet)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/BitSet.cpp
)

# Link application to libraries
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
