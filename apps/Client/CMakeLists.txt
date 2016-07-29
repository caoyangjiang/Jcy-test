cmake_minimum_required(VERSION 2.8)
project(Client)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Client.cpp

${PROJECT_SOURCE_DIR}/src/ChatMessage.h
)


find_package( Boost COMPONENTS system REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})

# Library for parsing Dash media presentation description

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

