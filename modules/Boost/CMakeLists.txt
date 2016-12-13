cmake_minimum_required(VERSION 3.5)
project(jcy_boost)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/SharedPtr.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/Boost/SharedPtr.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

find_package( Boost COMPONENTS system REQUIRED )

message(Check ${Boost_LIBRARIES})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})
target_link_libraries(${PROJECT_NAME} boost_system)

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")
