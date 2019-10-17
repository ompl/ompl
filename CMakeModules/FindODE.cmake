# - Find ODE (the Open Dynamics Engine)

include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(ODE ode)
    if(ODE_LIBRARIES AND NOT ODE_INCLUDE_DIRS)
        set(ODE_INCLUDE_DIRS "/usr/include")
    endif()
endif()
find_package_handle_standard_args(ODE DEFAULT_MSG ODE_LIBRARIES ODE_INCLUDE_DIRS)
