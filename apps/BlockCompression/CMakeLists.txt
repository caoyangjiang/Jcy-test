cmake_minimum_required(VERSION 3.5)
project(BlockCompression)

if(MSVC)
  add_definitions("-DHVR_WINDOWS_DLL_API=__declspec\(dllimport\)")
endif()


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/BlockCompression.cpp
)

# Find dependent libraries
find_package( SQUISH REQUIRED )
find_package( OpenCV REQUIRED )

# Include paths
include_directories(SYSTEM ${SQUISH_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

# Link application to libraries
target_link_libraries(${PROJECT_NAME} ${SQUISH_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})


