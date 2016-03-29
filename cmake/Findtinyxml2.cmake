# - Find tinyxml2
# Find the native tinyxml2 headers and libraries.
#
#  TINYXML2_INCLUDE_DIRS - where to find tinyxml2.h, etc.
#  TINYXML2_LIBRARIES    - List of libraries when using tinyxml2.
#  TINYXML2_FOUND        - True if tinyxml2 found.

if(DEFINED ENV{TINYXML2_ROOT_DIR})
    set(TINYXML2_ROOT_DIR "$ENV{TINYXML2_ROOT_DIR}")
endif()
set(TINYXML2_ROOT_DIR
    "${TINYXML2_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for tinyxml2")

# Look for the header file.
find_path(TINYXML2_INCLUDE_DIRS NAMES Tinyxml2 HINTS ${TINYXML2_ROOT_DIR})

# Look for the library.
find_library(TINYXML2_LIBRARY NAMES Tinyxml2
                              HINTS ${TINYXML2_ROOT_DIR}/build/Release)

mark_as_advanced(TINYXML2_LIBRARY)

set(TINYXML2_LIBRARIES ${TINYXML2_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set TINYXML2_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TINYXML2 DEFAULT_MSG
                                  TINYXML2_LIBRARIES TINYXML2_INCLUDE_DIRS)

mark_as_advanced(TINYXML2_LIBRARIES TINYXML2_INCLUDE_DIRS)
