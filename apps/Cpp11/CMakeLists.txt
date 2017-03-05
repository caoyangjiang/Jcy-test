cmake_minimum_required(VERSION 3.5)
project(Cpp11)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Cpp11.cpp
)

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

