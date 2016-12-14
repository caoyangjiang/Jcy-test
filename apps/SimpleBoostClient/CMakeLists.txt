cmake_minimum_required(VERSION 3.5)
project(SimpleBoostClient)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/SimpleBoostClient.cpp)

# Find dependent libraries
find_package( Boost COMPONENTS system REQUIRED )

# Include paths
include_directories(${Boost_INCLUDE_DIRS})

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

# Link application to libraries
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

