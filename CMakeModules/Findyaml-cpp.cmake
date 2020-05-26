# Find libyaml-cpp, a library for parsing YAML files

include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(YAMLCPP yaml-cpp)
    if(YAMLCPP_LIBRARIES AND NOT YAMLCPP_INCLUDE_DIRS)
        set(YAMLCPP_INCLUDE_DIRS "/usr/include")
    endif()
endif()
find_package_handle_standard_args(yaml-cpp DEFAULT_MSG YAMLCPP_LIBRARIES YAMLCPP_INCLUDE_DIRS)
