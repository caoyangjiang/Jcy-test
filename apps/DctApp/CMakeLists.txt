cmake_minimum_required(VERSION 3.7)
project(DctApp)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/DctApp.cpp
)

source_group("src" FILES ${PROJECT_SRCS})

find_package( Eigen3 REQUIRED )

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} jcy_transform)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
