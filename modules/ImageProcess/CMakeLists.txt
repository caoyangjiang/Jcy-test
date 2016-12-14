cmake_minimum_required(VERSION 3.5)
project(jcy_imageproc)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Convertor.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/ImageProc/Convertor.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

# Find dependent libraries
find_package( OpenCV REQUIRED )

# Include paths
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

# Compile module into library
add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

# Link module to external libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")
