cmake_minimum_required(VERSION 3.5)
project(Xml2)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Xml2.cpp
)

# Find dependent libraries
find_package( LibXml2 REQUIRED )

# Include paths
include_directories(${LIBXML2_INCLUDE_DIR})

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

# Link application to libraries
target_link_libraries(${PROJECT_NAME} ${LIBXML2_LIBRARIES})
