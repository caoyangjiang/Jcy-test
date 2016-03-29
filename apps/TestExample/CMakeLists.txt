cmake_minimum_required(VERSION 2.8)
project(TestExample)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/TestExample.cpp
)

include_directories(../../modules/Test/inc)

find_package( OpenCV REQUIRED )
# find_package( MATLAB REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${MATLAB_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${PROJECT_SRCS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
# target_link_libraries(${PROJECT_NAME} ${MATLAB_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Test)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
