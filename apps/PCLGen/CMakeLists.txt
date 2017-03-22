cmake_minimum_required(VERSION 3.7)
project(PCLGen)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/PCLGen.cpp
)

source_group("src" FILES ${PROJECT_SRCS})

find_package( PCL REQUIRED )

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
