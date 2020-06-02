# Find FLANN, a Fast Library for Approximate Nearest Neighbors

include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(FLANN flann)
    if(FLANN_LIBRARIES AND NOT FLANN_INCLUDE_DIRS)
        set(FLANN_INCLUDE_DIRS "/usr/include")
    endif()
endif()
find_package_handle_standard_args(flann DEFAULT_MSG FLANN_LIBRARIES FLANN_INCLUDE_DIRS)
