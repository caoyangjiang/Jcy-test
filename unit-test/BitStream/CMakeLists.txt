cmake_minimum_required(VERSION 3.7)
project(BitStreamTest)

if(MSVC)
  set(DLLIMPORT "__declspec(dllimport)")
  set(DLLEXPORT "__declspec(dllexport)")
  set(DISABLE "__pragma(warning(push,0))")
  set(ENABLE "__pragma(warning(pop))")
endif()

set(SAMPLE_INCS
${PROJECT_SOURCE_DIR}/include/BitStreamTest.h
)

set(SAMPLE_SRCS
${PROJECT_SOURCE_DIR}/src/BitStreamTest.cpp
${PROJECT_SOURCE_DIR}/src/main.cpp
)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996")
endif()

 
add_executable(BitStreamTest ${SAMPLE_SRCS} ${SAMPLE_INCS})

target_compile_definitions(${PROJECT_NAME} PUBLIC
                          JCY_WINDOWS_DISABLE_ALL_WARNING=${DISABLE}
                          JCY_WINDOWS_ENABLE_ALL_WARNING=${ENABLE})

target_include_directories(${PROJECT_NAME} PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

target_link_libraries(BitStreamTest jcy_misctools gtest_main)

set_property(TARGET BitStreamTest PROPERTY FOLDER ${CMAKE_PROJECT_NAME}/unit-test)
