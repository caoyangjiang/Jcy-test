cmake_minimum_required(VERSION 3.7)
project(MatrixTest)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/MatrixTest.cpp
)

source_group("src" FILES ${PROJECT_SRCS})

find_package( Boost REQUIRED )
find_package( Eigen3 REQUIRED )

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
                          ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} jcy_transform)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/apps)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}Targets
        DESTINATION bin)
