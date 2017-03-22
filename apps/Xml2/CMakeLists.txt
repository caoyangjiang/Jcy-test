cmake_minimum_required(VERSION 3.7)
project(Xml2)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Xml2.cpp
)

source_group("src" FILES ${PROJECT_SRCS})

find_package( LibXml2 REQUIRED )

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${LIBXML2_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} ${LIBXML2_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
