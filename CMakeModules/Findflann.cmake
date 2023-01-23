# Find FLANN, a Fast Library for Approximate Nearest Neighbors

include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(FLANN flann)
    if(FLANN_LIBRARIES AND NOT FLANN_INCLUDE_DIRS)
        set(FLANN_INCLUDE_DIRS "/usr/include")
    endif()

    if (FLANN_LIBRARY_DIRS)
      foreach(FDIR ${FLANN_LIBRARY_DIRS})
        string(REGEX MATCH "^-l" FOUND ${FDIR})
        if (FOUND)
          string(REGEX REPLACE "^-l" "" FDIR2 ${FDIR})
          list(PREPEND FLANN_LIBRARIES ${FDIR2})
        else()
          list(APPEND FLANN_LIBRARY_DIRS_TEMP ${FDIR})
        endif()
      endforeach()

      set(FLANN_LIBRARY_DIRS ${FLANN_LIBRARY_DIRS_TEMP})
    endif()
endif()

find_package_handle_standard_args(flann DEFAULT_MSG FLANN_LIBRARY_DIRS FLANN_LIBRARIES FLANN_INCLUDE_DIRS)
