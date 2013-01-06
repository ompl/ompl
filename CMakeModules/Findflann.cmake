# Find FLANN, a Fast Library for Approximate Nearest Neighbors

include(FindPackageHandleStandardArgs)

find_path(FLANN_INCLUDE_DIRS flann.hpp PATH_SUFFIXES flann)
if (FLANN_INCLUDE_DIRS)
    file(READ "${FLANN_INCLUDE_DIRS}/config.h" FLANN_CONFIG)
    string(REGEX REPLACE ".*FLANN_VERSION_ \"([0-9.]+)\".*" "\\1" FLANN_VERSION ${FLANN_CONFIG})
    if(FLANN_VERSION VERSION_LESS flann_FIND_VERSION)
        unset(FLANN_INCLUDE_DIRS)
    else()
        string(REGEX REPLACE "/flann$" "" FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIRS})
    endif()
endif()

find_package_handle_standard_args(flann DEFAULT_MSG FLANN_INCLUDE_DIRS)
