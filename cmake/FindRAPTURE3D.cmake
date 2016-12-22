# - Find Rapture3D Universal SDK
# Find the native Rapturee3D universal SDK headers and libraries.
#
#  RAPTURE3D_INCLUDE_DIR - where to find SDL.h, etc.
#  RAPTURE3D_LIBRARIES   - List of libraries when using Rapture3D.
#  RAPTURE3D_ROOT_DIR    - The base directory to search for Rapture3D.
#                          This can also be an environment variable.

if(DEFINED ENV{RAPTURE3D_ROOT_DIR})
    set(RAPTURE3D_ROOT_DIR "$ENV{RAPTURE3D_ROOT_DIR}")
endif()
set(RAPTURE3D_ROOT_DIR
    "${RAPTURE3D_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for Rapture3D")

# Look for the header file.
find_path(RAPTURE3D_INCLUDE_DIR NAMES R3D/r3d.h HINTS
  ${RAPTURE3D_ROOT_DIR}/include
  /usr/local/include/)

# Look for the library.
find_library(RAPTURE3D_LIBRARY NAMES R3DU
              HINTS ${RAPTURE3D_ROOT_DIR}/lib/x64
              HINTS /usr/local/lib/)

mark_as_advanced(RAPTURE3D_LIBRARY)

#set(RAPTURE3D_LIBRARIES ${RAPTURE3D_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set RAPTURE3D_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RAPTURE3D DEFAULT_MSG RAPTURE3D_LIBRARY
                                  RAPTURE3D_INCLUDE_DIR)
