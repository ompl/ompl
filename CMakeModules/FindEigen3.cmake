include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN3 eigen3)
endif()

find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIRS)
