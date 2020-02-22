# CMake OMPL module
#
# It defines the following variables:
# OMPL_FOUND         - TRUE
# OMPL_INCLUDE_DIRS  - The OMPL include directory
# OMPL_LIBRARIES     - The OMPL library
# OMPLAPP_LIBRARIES  - The OMPL.app libraries (if installed)
# OMPL_VERSION       - The OMPL version in the form <major>.<minor>.<patchlevel>
# OMPL_MAJOR_VERSION - Major version
# OMPL_MINOR_VERSION - Minor version
# OMPL_PATCH_VERSION - Patch version


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was omplConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

set(OMPL_VERSION 1.5.0)
set(OMPL_MAJOR_VERSION 1)
set(OMPL_MINOR_VERSION 5)
set(OMPL_PATCH_VERSION 0)

set_and_check(OMPL_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set(OMPL_INCLUDE_DIRS "${OMPL_INCLUDE_DIR};/usr/include/eigen3;;")
list(REMOVE_DUPLICATES OMPL_INCLUDE_DIRS)
list(REMOVE_ITEM OMPL_INCLUDE_DIRS "")
set(OMPL_INCLUDE_DIRS "${OMPL_INCLUDE_DIRS}" CACHE STRING "Include path for OMPL and its dependencies")

set_and_check(OMPL_LIBRARY_DIR ${PACKAGE_PREFIX_DIR}/lib)
find_library(OMPL_LIBRARIES NAMES ompl.${OMPL_VERSION} ompl
    PATHS ${OMPL_LIBRARY_DIR} NO_DEFAULT_PATH)
find_library(OMPLAPPBASE_LIBRARY NAMES ompl_app_base.${OMPL_VERSION} ompl_app_base
    PATHS ${OMPL_LIBRARY_DIR} NO_DEFAULT_PATH)
find_library(OMPLAPP_LIBRARY NAMES ompl_app.${OMPL_VERSION} ompl_app
    PATHS ${OMPL_LIBRARY_DIR} NO_DEFAULT_PATH)
if (OMPLAPPBASE_LIBRARY AND OMPLAPP_LIBRARY)
    set(OMPLAPP_LIBRARIES "${OMPLAPPBASE_LIBRARY};${OMPLAPP_LIBRARY}"
        CACHE STRING "Paths to OMPL.app libraries")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ompl DEFAULT_MSG OMPL_INCLUDE_DIRS OMPL_LIBRARIES)
