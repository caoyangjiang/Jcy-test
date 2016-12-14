cmake_minimum_required(VERSION 3.5)
project(jcy_transform)

# Define dllexport for public methods on Visual Studio for building library
if(MSVC)
  add_definitions(/D "HVR_WINDOWS_DLL_API=__declspec(dllexport)")
endif()

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/Dct.cpp
${PROJECT_SOURCE_DIR}/src/SADct.cpp
${PROJECT_SOURCE_DIR}/src/Dct2d.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/Transform/Dct.h
${PROJECT_SOURCE_DIR}/include/Jcy/Transform/SADct.h
${PROJECT_SOURCE_DIR}/include/Jcy/Transform/Dct2d.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

# Find dependent libraries
find_package( Eigen3 REQUIRED )

# Include paths
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# Compile module into library
add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

# Group this project into "Jcy-test/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")
