# - Find Matlab
# Find the native Matlab headers and libraries.
#
#  MATLAB_INCLUDE_DIRS - where to find mat.h, etc.
#  MATLAB_LIBRARIES    - List of libraries when using Matlab.
#  MATLAB_FOUND        - True if Matlab found.

if(DEFINED ENV{MATLAB_ROOT_DIR})
    set(MATLAB_ROOT_DIR "$ENV{MATLAB_ROOT_DIR}")
endif()
set(MATLAB_ROOT_DIR
    "${MATLAB_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for Matlab")

# Look for the header file.
find_path(MATLAB_INCLUDE_DIR NAMES mat.h HINTS
    ${MATLAB_ROOT_DIR}/extern/include
  /usr/local/MATLAB/MATLAB_Runtime/v90/extern/include
    /usr/local/MATLAB/R2014a/extern/include
    /usr/local/MATLAB/R2014b/extern/include
    /usr/local/MATLAB/R2015a/extern/include)

# Look for the library.
find_library(MAT_LIBRARY
              NAMES mat libmat
              HINTS /usr/local/MATLAB/MATLAB_Runtime/v90/bin/glnxa64
                    ${MATLAB_ROOT_DIR}/bin/glnxa64
                    ${MATLAB_ROOT_DIR}/extern/lib/win64/microsoft
                    /usr/local/MATLAB/R2014a/bin/glnxa64
                    /usr/local/MATLAB/R2014b/bin/glnxa64
                    /usr/local/MATLAB/R2015a/bin/glnxa64)

mark_as_advanced(MAT_LIBRARY)

find_library(MX_LIBRARY
              NAMES mx libmx
              HINTS /usr/local/MATLAB/MATLAB_Runtime/v90/bin/glnxa64
                    ${MATLAB_ROOT_DIR}/extern/lib/win64/microsoft
                    /usr/local/MATLAB/R2014a/bin/glnxa64
                    /usr/local/MATLAB/R2014b/bin/glnxa64
                    /usr/local/MATLAB/R2015a/bin/glnxa64)

mark_as_advanced(MX_LIBRARY)

set(MATLAB_LIBRARIES ${MAT_LIBRARY} ${MX_LIBRARY})

set(MATLAB_INCLUDE_DIRS ${MATLAB_INCLUDE_DIR})

# handle the QUIETLY and REQUIRED arguments and set MATLAB_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MATLAB DEFAULT_MSG MATLAB_LIBRARIES
                                  MATLAB_INCLUDE_DIRS)

mark_as_advanced(MATLAB_LIBRARIES MATLAB_INCLUDE_DIRS)
