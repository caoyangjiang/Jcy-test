cmake_minimum_required(VERSION 3.5)
project(jcy_eigen)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Dct.cpp
${PROJECT_SOURCE_DIR}/src/SADct.cpp
${PROJECT_SOURCE_DIR}/src/Dct2d.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/Eigen/Dct.h
${PROJECT_SOURCE_DIR}/include/Jcy/Eigen/SADct.h
${PROJECT_SOURCE_DIR}/include/Jcy/Eigen/Dct2d.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

find_package( Eigen3 REQUIRED )

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")
