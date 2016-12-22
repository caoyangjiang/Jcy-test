# - Find X264
# Find the native X264 headers and libraries.
#
#  X264_INCLUDE_DIRS - where to find x264.h, etc.
#  X264_LIBRARIES    - List of libraries when using x264.
#  X264_FOUND        - True if x264 found.

if(DEFINED ENV{X264_ROOT_DIR})
    set(X264_ROOT_DIR "$ENV{X264_ROOT_DIR}")
endif()
set(X264_ROOT_DIR
    "${X264_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for Matlab")

# Look for the header file.
find_path(X264_INCLUDE_DIR NAMES x264.h HINTS
  ${X264_ROOT_DIR}/include
  /usr/include)

# Look for the library.
find_library(X264_LIBRARY NAMES x264 libx264 HINTS
  ${X264_ROOT_DIR}/lib
  /usr/lib/x86_64-linux-gnu/)

mark_as_advanced(X264_LIBRARY)

set(X264_LIBRARIES ${X264_LIBRARY})

set(X264_INCLUDE_DIRS ${X264_INCLUDE_DIR})

# handle the QUIETLY and REQUIRED arguments and set X264_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(X264 DEFAULT_MSG X264_LIBRARIES
                                  X264_INCLUDE_DIRS)

mark_as_advanced(X264_LIBRARIES X264_INCLUDE_DIRS)
