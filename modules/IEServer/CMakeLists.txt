cmake_minimum_required(VERSION 3.5)
project(jcy_ieserver)

# Define dllexport for public methods on Visual Studio for building library
if(MSVC)
  add_definitions(/D "HVR_WINDOWS_DLL_API=__declspec(dllexport)")
endif()


set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ChatClient.cpp
${PROJECT_SOURCE_DIR}/src/ChatServer.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/IEServer/ChatClient.h
${PROJECT_SOURCE_DIR}/include/Jcy/IEServer/ChatMessage.h
${PROJECT_SOURCE_DIR}/include/Jcy/IEServer/ChatServer.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

find_package( Boost COMPONENTS system REQUIRED )

# Include paths
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

# Compile module into library
add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")
