# - Find glog
# Find the native glog headers and libraries.
#
#  GLOG_INCLUDE_DIRS - where to find logging.h, etc.
#  GLOG_LIBRARIES    - List of libraries when using glog.
#  GLOG_FOUND        - True if glog found.

if(DEFINED ENV{GLOG_ROOT_DIR})
    set(GLOG_ROOT_DIR "$ENV{GLOG_ROOT_DIR}")
endif()
set(GLOG_ROOT_DIR
    "${GLOG_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for glog")

# Look for the header file.
find_path(GLOG_INCLUDE_DIRS NAMES glog/logging.h HINTS
        ${GLOG_ROOT_DIR}/src/windows)

# Look for the library.
find_library(GLOG_LIBRARY NAMES libglog glog
            HINTS ${GLOG_ROOT_DIR}/x64/Release)

mark_as_advanced(GLOG_LIBRARY)

set(GLOG_LIBRARIES ${GLOG_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set GLOG_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLOG DEFAULT_MSG GLOG_LIBRARIES
                                GLOG_INCLUDE_DIRS)

mark_as_advanced(GLOG_LIBRARIES GLOG_INCLUDE_DIRS)
