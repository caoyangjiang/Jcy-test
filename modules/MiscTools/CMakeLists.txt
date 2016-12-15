cmake_minimum_required(VERSION 3.5)
project(jcy_misctools)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/src/ObjFile.cpp
${PROJECT_SOURCE_DIR}/src/CloudReader.cpp
${PROJECT_SOURCE_DIR}/src/HvrMeshContainer.cpp
${PROJECT_SOURCE_DIR}/src/DdsWriter.cpp
${PROJECT_SOURCE_DIR}/src/BitStream.cpp
)

set(PROJECT_INCS
${PROJECT_SOURCE_DIR}/include/Jcy/MiscTools/ObjFile.h
${PROJECT_SOURCE_DIR}/include/Jcy/MiscTools/CloudReader.h
${PROJECT_SOURCE_DIR}/include/Jcy/MiscTools/HvrMeshContainer.h
${PROJECT_SOURCE_DIR}/include/Jcy/MiscTools/DdsWriter.h
${PROJECT_SOURCE_DIR}/include/Jcy/MiscTools/BitStream.h
)

# Separate include and src folder on Visual Studio
source_group("include" FILES ${PROJECT_INCS})
source_group("src"     FILES ${PROJECT_SRCS})

# Find dependent libraries
find_package( PCL REQUIRED )

# Include paths
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

# Compile module into library
add_library(${PROJECT_NAME} SHARED ${PROJECT_SRCS} ${PROJECT_INCS})

# Link module to external libraries
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# Group this project into "hvr-decode/modules" in Visual Studio
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Jcy-test/modules")
