# - Find MM3D
# Find the native MM3D headers and libraries.
#
#  MM3D_INCLUDE_DIRS - where to find SDL.h, etc.
#  MM3D_LIBRARIES    - List of libraries when using MM3D.
#  MM3D_ROOT_DIR     - The base directory to search for MM3D.
#                      This can also be an environment variable.

if(DEFINED ENV{MM3D_ROOT_DIR})
    set(MM3D_ROOT_DIR "$ENV{MM3D_ROOT_DIR}")
endif()
set(MM3D_ROOT_DIR
    "${MM3D_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for MM3D")

# Look for the header file.
find_path(MM3D_INCLUDE_DIRS NAMES liondance.h HINTS
  ${MM3D_ROOT_DIR}/include
  /usr/local/include/)

# Look for the library.
find_library(MM3D_LIBRARY NAMES liondance
              HINTS ${MM3D_ROOT_DIR}/lib
              HINTS /usr/local/lib)

mark_as_advanced(MM3D_LIBRARY)

set(MM3D_LIBRARIES ${MM3D_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set MM3D_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MM3D DEFAULT_MSG MM3D_LIBRARIES
                                  MM3D_INCLUDE_DIRS)
