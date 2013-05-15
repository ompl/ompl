# Find FLANN, a Fast Library for Approximate Nearest Neighbors

include(FindPackageHandleStandardArgs)

find_path(FLANN_INCLUDE_DIR flann.hpp PATH_SUFFIXES flann)
if (FLANN_INCLUDE_DIR)
    file(READ "${FLANN_INCLUDE_DIR}/config.h" FLANN_CONFIG)
    string(REGEX REPLACE ".*FLANN_VERSION_ \"([0-9.]+)\".*" "\\1" FLANN_VERSION ${FLANN_CONFIG})
    if(NOT FLANN_VERSION VERSION_LESS flann_FIND_VERSION)
        string(REGEX REPLACE "/flann$" "" FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
    endif()
endif()

find_package_handle_standard_args(flann DEFAULT_MSG FLANN_INCLUDE_DIRS)
