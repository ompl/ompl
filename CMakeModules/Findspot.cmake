# Find spot, a platform for LTL and ω-automata manipulation
# See https://spot.lrde.epita.fr

include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(SPOT libspot)
    if(SPOT_LIBRARIES AND NOT SPOT_INCLUDE_DIRS)
        set(SPOT_INCLUDE_DIRS "/usr/include")
    endif()
endif()
find_package_handle_standard_args(spot DEFAULT_MSG SPOT_LIBRARIES SPOT_INCLUDE_DIRS)
