cmake_minimum_required(VERSION 3.5)
project(PCLGen)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/PCLGen.cpp
)

find_package( PCL REQUIRED )

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
include_directories(../../modules/PCL/include)

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

#target_link_libraries(${PROJECT_NAME} jcy_eigen)

