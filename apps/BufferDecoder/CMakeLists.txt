cmake_minimum_required(VERSION 3.7)
project(BufferDecoder)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/BufferDecoder.cpp
)

source_group("src" FILES ${PROJECT_SRCS})

find_package( FFMPEG REQUIRED )
 
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${FFMPEG_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
