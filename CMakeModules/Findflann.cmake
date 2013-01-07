# Find FLANN, a Fast Library for Approximate Nearest Neighbors

include(FindPackageHandleStandardArgs)

find_path(FLANN_INCLUDE_DIR flann.hpp PATH_SUFFIXES flann)
if (FLANN_INCLUDE_DIR)
    file(READ "${FLANN_INCLUDE_DIR}/config.h" FLANN_CONFIG)
    string(REGEX REPLACE ".*FLANN_VERSION_ \"([0-9.]+)\".*" "\\1" FLANN_VERSION ${FLANN_CONFIG})
    if(FLANN_VERSION VERSION_LESS flann_FIND_VERSION)
        message(STATUS "Installed version of flann is too old. Version ${flann_FIND_VERSION} or above required")
    else()
        string(REGEX REPLACE "/flann$" "" FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
    endif()
endif()
message("FLANN_INCLUDE_DIRS = ${FLANN_INCLUDE_DIRS}")
find_package_handle_standard_args(flann DEFAULT_MSG FLANN_INCLUDE_DIRS)
