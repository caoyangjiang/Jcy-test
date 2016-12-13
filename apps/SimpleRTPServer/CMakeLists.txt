cmake_minimum_required(VERSION 3.5)
project(SimpleRTPServer)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/SimpleRTPServer.cpp)

find_package( FFMPEG REQUIRED )

include_directories(${FFMPEG_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})
