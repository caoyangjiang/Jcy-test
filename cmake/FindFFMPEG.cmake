# - Find FFmpeg
# Find the native ffmpeg headers and libraries.
#
#  FFMPEG_INCLUDE_DIRS - where to find avcodec.h, etc.
#  FFMPEG_LIBRARIES    - List of libraries when using ffmpeg.
#  FFMPEG_ROOT_DIR     - The base directory to search for ffmpeg.
#                       This can also be an environment variable.


if(DEFINED ENV{FFMPEG_ROOT_DIR})
    set(FFMPEG_ROOT_DIR "$ENV{FFMPEG_ROOT_DIR}")
endif()
set(FFMPEG_ROOT_DIR
    "${FFMPEG_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for FFMPEG")

# Look for the header file.
find_path(FFMPEG_INCLUDE_DIR NAMES libavcodec/avcodec.h HINTS
  ${FFMPEG_ROOT_DIR}/include
  /usr/local/include/)

# Look for the library. (Order matters. DO NOT change order!)
find_library(FFMPEG_AVFORMAT_LIBRARY NAMES avformat
  HINTS ${FFMPEG_ROOT_DIR}/lib
  /usr/local/lib)
mark_as_advanced(FFMPEG_AVFORMAT_LIBRARY)

find_library(FFMPEG_AVUTIL_LIBRARY NAMES avutil
  HINTS ${FFMPEG_ROOT_DIR}/lib
  /usr/local/lib)
mark_as_advanced(FFMPEG_AVUTIL_LIBRARY)

find_library(FFMPEG_AVCODEC_LIBRARY NAMES avcodec
  HINTS ${FFMPEG_ROOT_DIR}/lib
  /usr/local/lib)

mark_as_advanced(FFMPEG_AVCODEC_LIBRARY)

set(FFMPEG_LIBRARIES ${FFMPEG_AVFORMAT_LIBRARY}
  ${FFMPEG_AVUTIL_LIBRARY}
  ${FFMPEG_AVCODEC_LIBRARY})
set(FFMPEG_INCLUDE_DIRS ${FFMPEG_INCLUDE_DIR})

# handle the QUIETLY and REQUIRED arguments and set FFMPEG_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FFMPEG DEFAULT_MSG FFMPEG_LIBRARIES
                                  FFMPEG_INCLUDE_DIRS)
