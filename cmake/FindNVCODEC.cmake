# - Find NVCodec
# Find the NVCodec headers and libraries.
#
#  NVCODEC_INCLUDE_DIRS - where to find nvcuvid.h, cuviddec.h, and nvEncodeAPI.h
#  NVCODEC_LIBRARIES    - List of libraries when using NVCODEC
#  NVCODEC_ROOT_DIR     - The base directory to search for NVCODEC.
#                         This can also be an environment variable.

if(DEFINED ENV{NVCODEC_ROOT_DIR})
    set(NVCODEC_ROOT_DIR "$ENV{NVCODEC_ROOT_DIR}")
endif()
set(NVCODEC_ROOT_DIR
    "${NVCUVID_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for NVCODEC")


# Look for the header file.
find_path(NVCODEC_INCLUDE_DIRS
  NAMES
  nvcuvid.h
  cuviddec.h
  nvEncodeAPI.h
  HINTS
  ${NVCUVID_ROOT_DIR}/ #nvcuvid.h, cuviddec.h on Linux
  ${NVCODEC_ROOT_DIR}/ #nvEncodeAPI.h path on Windows
  /usr/local/include) #nvucuvid.h, cuviddec.h and nvEncodeAPI.h path on Linux


find_library(NVCUVID_LIBRARY NAMES nvcuvid
  HINTS
  ${NVCUVID_ROOT_DIR}/lib/x64 #On windows, nvcuvid is in CUDA SDK
  /usr/lib64/nvidia
  /usr/lib/x86_64-linux-gnu) #On Linux, nvcuvid is in NVIDIA driver

mark_as_advanced(NVCUVID_LIBRARY)

find_library(NVIDIAENCODE_LIBRARY NAMES nvidia-encode
  HINTS
  /usr/lib64/nvidia
  /usr/lib/x86_64-linux-gnu)

mark_as_advanced(NVIDIAENCODE_LIBRARY)

find_library(CU_LIBRARY NAMES cuda
  HINTS
  ${NVCUVID_ROOT_DIR}/lib/x64
  /usr/lib/x86_64-linux-gnu)

mark_as_advanced(CU_LIBRARY)

find_library(NVCUVID_CUDART_STATIC_LIBRARY NAMES cudart
  HINTS
  ${NVCUVID_ROOT_DIR}/lib/x64
  /usr/local/cuda/lib64/)

mark_as_advanced(NVCUVID_CUDART_STATIC_LIBRARY)

set(NVCODEC_LIBRARIES
    ${NVCUVID_LIBRARY}
    ${NVIDIAENCODE_LIBRARY}
    ${CU_LIBRARY}
    ${NVCUVID_CUDART_STATIC_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set NVCUVID_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NVCODEC DEFAULT_MSG NVCODEC_LIBRARIES
                                  NVCODEC_INCLUDE_DIRS)
