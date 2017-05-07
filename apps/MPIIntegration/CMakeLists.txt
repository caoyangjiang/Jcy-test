cmake_minimum_required(VERSION 3.7)
project(MPIIntegration)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/MPIIntegration.cpp
)

source_group("src" FILES ${PROJECT_SRCS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

find_package( MPI REQUIRED )

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${MPI_INCLUDE_PATH})

target_link_libraries(${PROJECT_NAME} ${MPI_LIBRARIES} jcy_hpctool)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
