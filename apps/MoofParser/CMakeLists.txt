cmake_minimum_required(VERSION 3.7)
project(MoofParser)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/MoofParser.cpp
)

source_group("src" FILES ${PROJECT_SRCS})
 
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
