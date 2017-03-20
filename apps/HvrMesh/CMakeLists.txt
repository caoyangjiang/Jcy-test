cmake_minimum_required(VERSION 3.7)
project(HvrMesh)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/HvrMesh.cpp
)

source_group("src" FILES ${PROJECT_SRCS})

# Compile project into application
add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

# Link application to libraries
target_link_libraries(${PROJECT_NAME} jcy_misctools)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
