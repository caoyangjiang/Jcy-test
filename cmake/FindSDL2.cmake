# - Find SDL2
# Find the native SDL2 headers and libraries.
#
#  SDL2_INCLUDE_DIRS - where to find SDL.h, etc.
#  SDL2_LIBRARIES    - List of libraries when using SDL2.
#  SDL2_ROOT_DIR     - The base directory to search for SDL2.
#                      This can also be an environment variable.

if(DEFINED ENV{SDL2_ROOT_DIR})
    set(SDL2_ROOT_DIR "$ENV{SDL2_ROOT_DIR}")
endif()
set(SDL2_ROOT_DIR
    "${SDL2_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for SDL2")

# Look for the header file.
find_path(SDL2_INCLUDE_DIRS NAMES SDL.h HINTS
  ${SDL2_ROOT_DIR}/include
  /usr/include/SDL2)

# Look for the library.
find_library(SDL2_LIBRARY NAMES SDL2
              HINTS ${SDL2_ROOT_DIR}/lib/x64
              HINTS /usr/lib/x86_64-linux-gnu)

mark_as_advanced(SDL2_LIBRARY)

find_library(SDL2_MAIN_LIBRARY NAMES SDL2main
              HINTS ${SDL2_ROOT_DIR}/lib/x64
              HINTS /usr/lib/x86_64-linux-gnu)

mark_as_advanced(SDL2_MAIN_LIBRARY)

set(SDL2_LIBRARIES ${SDL2_LIBRARY} ${SDL2_MAIN_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set SDL2_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SDL2 DEFAULT_MSG SDL2_LIBRARIES
                                  SDL2_INCLUDE_DIRS)
