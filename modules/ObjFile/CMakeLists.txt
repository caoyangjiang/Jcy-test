cmake_minimum_required(VERSION 2.8)
project(jcy_objfile)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ObjFile.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/ObjFile/ObjFile.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")