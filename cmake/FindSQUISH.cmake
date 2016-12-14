# - Find Squish
# Find the native squish headers and libraries.
#
#  SQUISH_INCLUDE_DIRS - where to find squish.h, etc.
#  SQUISH_LIBRARIES    - List of libraries when using squish.
#  SQUISH_FOUND        - True if squish found.

if(DEFINED ENV{SQUISH_ROOT_DIR})
    set(SQUISH_ROOT_DIR "$ENV{SQUISH_ROOT_DIR}")
endif()
set(SQUISH_ROOT_DIR
    "${SQUISH_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for Matlab")

# Look for the header file.
find_path(SQUISH_INCLUDE_DIR NAMES squish.h HINTS
  ${SQUISH_ROOT_DIR}/include
  /usr/local/include)

# Look for the library.
find_library(SQUISH_LIBRARY NAMES squish libsquish squishd HINTS
  ${SQUISH_ROOT_DIR}/lib
  /usr/local/lib)

mark_as_advanced(SQUISH_LIBRARY)

set(SQUISH_LIBRARIES ${SQUISH_LIBRARY})

set(SQUISH_INCLUDE_DIRS ${SQUISH_INCLUDE_DIR})

# handle the QUIETLY and REQUIRED arguments and set SQUISH_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SQUISH DEFAULT_MSG SQUISH_LIBRARIES
                                  SQUISH_INCLUDE_DIRS)

mark_as_advanced(SQUISH_LIBRARIES SQUISH_INCLUDE_DIRS)
