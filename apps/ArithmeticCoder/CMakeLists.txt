cmake_minimum_required(VERSION 3.5)
project(ArithmeticCoder)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ArithmeticCoder.cpp
)

# Include paths
include_directories(../../modules/EntropyCoder/include)
include_directories(../../modules/MiscTools/include)

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

# Link application to libraries
target_link_libraries(${PROJECT_NAME} jcy_entropycoder)
