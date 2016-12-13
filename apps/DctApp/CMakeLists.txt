cmake_minimum_required(VERSION 3.5)
project(DctApp)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/DctApp.cpp
)

find_package( Eigen3 REQUIRED )

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(../../modules/Eigen/include)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
target_link_libraries(${PROJECT_NAME} jcy_eigen)

