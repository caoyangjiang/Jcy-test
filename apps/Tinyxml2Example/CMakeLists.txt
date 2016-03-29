cmake_minimum_required(VERSION 2.8)
project(Tinyxml2Example)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Tinyxml2Example.cpp
)

if(MSVC)
  find_package( tinyxml2 REQUIRED )
endif()

if(MSVC)
  include_directories(${TINYXML2_INCLUDE_DIRS})
endif()

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

if(MSVC)
  target_link_libraries(${PROJECT_NAME} ${TINYXML2_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} tinyxml2)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
