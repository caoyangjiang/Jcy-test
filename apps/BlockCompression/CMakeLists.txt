cmake_minimum_required(VERSION 3.7)
project(BlockCompression)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/BlockCompression.cpp
)

source_group("src" FILES ${PROJECT_SRCS})

find_package( SQUISH REQUIRED )
find_package( OpenCV REQUIRED )

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${SQUISH_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${SQUISH_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
