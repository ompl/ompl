# Find spot, a platform for LTL and ω-automata manipulation
# See https://spot.lrde.epita.fr

include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(SPOT libspot
        IMPORTED_TARGET)
    if(SPOT_LIBRARIES AND NOT SPOT_INCLUDE_DIRS)
        set(SPOT_INCLUDE_DIRS "/usr/include")
    endif()
endif()
find_package_handle_standard_args(spot DEFAULT_MSG SPOT_LIBRARIES SPOT_INCLUDE_DIRS)

if(spot_FOUND AND NOT TARGET Spot::Spot)
    set_target_properties(PkgConfig::SPOT PROPERTIES
        IMPORTED_LOCATION "${SPOT_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${SPOT_INCLUDE_DIRS}"
    )
    add_library(Spot::Spot ALIAS PkgConfig::SPOT)
endif()
