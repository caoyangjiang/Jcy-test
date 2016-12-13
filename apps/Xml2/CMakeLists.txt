cmake_minimum_required(VERSION 3.5)
project(Xml2)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Xml2.cpp
)
# Library for parsing Dash media presentation description
find_package( LibXml2 REQUIRED )

include_directories(${LIBXML2_INCLUDE_DIR})
message(${LIBXML2_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} ${LIBXML2_LIBRARIES})
