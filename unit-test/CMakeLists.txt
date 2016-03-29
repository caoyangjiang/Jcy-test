cmake_minimum_required(VERSION 2.8)
project(unit-test)

if(MSVC)
  set(gtest_force_shared_crt ON)
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-inline")
endif()

# Download and unpack gtest at configure time
configure_file(gtest.txt gtest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtest-download)
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtest-download)

# Add gtest directly to our build
add_subdirectory(${CMAKE_BINARY_DIR}/gtest-src
                ${CMAKE_BINARY_DIR}/gtest-build
                EXCLUDE_FROM_ALL)

# Add the gtest include directory, since gtest
# doesn't add that dependency to its gtest target
target_include_directories(gtest INTERFACE
                          "${CMAKE_BINARY_DIR}/gtest-src/include")


if(NOT TARGET test)
  add_custom_target(test
                    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/EigenTest/EigenTest)
endif()

add_subdirectory(EigenTest)
