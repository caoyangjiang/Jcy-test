# - Find Nvcuvid
# Find the native NVCUVID headers and libraries.
#
#  NVCUVID_INCLUDE_DIRS - where to find avcodec.h, etc.
#  NVCUVID_LIBRARIES    - List of libraries when using NVCUVID.
#  NVCUVID_ROOT_DIR     - The base directory to search for NVCUVID.
#                         This can also be an environment variable.

if(DEFINED ENV{NVCUVID_ROOT_DIR})
    set(NVCUVID_ROOT_DIR "$ENV{NVCUVID_ROOT_DIR}")
endif()
set(NVCUVID_ROOT_DIR
    "${NVCUVID_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for NVCUVID")

# Look for the header file.
find_path(NVCUVID_INCLUDE_DIR NAMES nvcuvid.h cuviddec.h
  HINTS
  ${NVCUVID_ROOT_DIR}/include
  /usr/local/include
  /usr/local/cuda/include)

# Look for the library. (Order matters. DO NOT change order!)
find_library(NVCUVID_LIBRARY NAMES nvcuvid
  HINTS ${NVCUVID_ROOT_DIR}/lib/x64
  /usr/lib64/nvidia
  /usr/lib/x86_64-linux-gnu)

mark_as_advanced(NVCUVID_LIBRARY)

find_library(CU_LIBRARY NAMES cuda
  HINTS
  ${NVCUVID_ROOT_DIR}/lib/x64
  /usr/lib/x86_64-linux-gnu)

mark_as_advanced(CU_LIBRARY)

find_library(NVCUVID_CUDART_STATIC_LIBRARY NAMES cudart
  HINTS
  ${NVCUVID_ROOT_DIR}/lib/x64
  /usr/lib/x86_64-linux-gnu
  /usr/local/cuda/lib64/)

mark_as_advanced(NVCUVID_CUDART_STATIC_LIBRARY)

set(NVCUVID_LIBRARIES
    ${NVCUVID_LIBRARY}
    ${CU_LIBRARY}
    ${NVCUVID_CUDART_STATIC_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set NVCUVID_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NVCUVID DEFAULT_MSG NVCUVID_LIBRARIES
                                  NVCUVID_INCLUDE_DIR)
