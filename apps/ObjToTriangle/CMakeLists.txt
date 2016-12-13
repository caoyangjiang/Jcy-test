cmake_minimum_required(VERSION 3.5)
project(ObjToTriangle)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ObjToTriangle.cpp
)

find_package( PCL REQUIRED )

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})


