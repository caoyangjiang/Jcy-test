cmake_minimum_required(VERSION 2.8)
project(jcy_pcl)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/CloudReader.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/Pcl/CloudReader.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

find_package( PCL REQUIRED )

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")