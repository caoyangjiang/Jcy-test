cmake_minimum_required(VERSION 3.5)
project(HvrMesh)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/HvrMesh.cpp
)

find_package( Eigen3 REQUIRED )

include_directories(../../modules/MiscTools/include)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} jcy_misctools)
