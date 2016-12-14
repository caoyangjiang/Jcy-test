cmake_minimum_required(VERSION 3.5)
project(DctApp)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/DctApp.cpp
)

# Find dependent libraries
find_package( Eigen3 REQUIRED )

# Include paths
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(../../modules/Transform/include)

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

# Link application to libraries
target_link_libraries(${PROJECT_NAME} jcy_transform)

