cmake_minimum_required(VERSION 3.5)
project(jcy_opencv)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Convertor.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/OpenCV/Convertor.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

find_package( OpenCV REQUIRED )

include_directories(${PROJECT_SOURCE_DIR}/include)
#message(hello${OpenCV_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")
