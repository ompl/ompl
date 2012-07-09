# - Try to find the OMPL library
# Once done this will define:
#
# OMPL_FOUND - OMPL was found
# OMPL_INCLUDE_DIRS - The OMPL include directory
# OMPL_LIBRARIES - The OMPL library
# OMPLAPP_LIBRARIES - The OMPL.app library

include(FindPackageHandleStandardArgs)

# user can set OMPL_PREFIX to specify the prefix path of the OMPL library
# and include directory, either as an environment variable or as an
# argument to cmake ("cmake -DOMPL_PREFIX=...")
if (NOT OMPL_PREFIX)
    set(OMPL_PREFIX $ENV{OMPL_PREFIX})
endif()

# user can set OMPL_LIB_PATH to specify the path for the OMPL library
# (analogous to OMPL_PREFIX)
if (NOT OMPL_LIB_PATH)
    set(OMPL_LIB_PATH $ENV{OMPL_LIB_PATH})
    if (NOT OMPL_LIB_PATH)
        set(OMPL_LIB_PATH ${OMPL_PREFIX})
    endif()
endif()

# user can set OMPL_INCLUDE_PATH to specify the path for the OMPL include
# directory (analogous to OMPL_PREFIX)
if (NOT OMPL_INCLUDE_PATH)
    set(OMPL_INCLUDE_PATH $ENV{OMPL_INCLUDE_PATH})
    if (NOT OMPL_INCLUDE_PATH)
        set(OMPL_INCLUDE_PATH ${OMPL_PREFIX})
    endif()
endif()


# find the OMPL library
find_library(OMPL_LIBRARIES ompl
    PATHS ${OMPL_LIB_PATH}
    PATH_SUFFIXES lib build/lib)
# find the OMPL library
find_library(OMPLAPP_LIBRARIES ompl_app
    PATHS ${OMPL_LIB_PATH}
    PATH_SUFFIXES lib build/lib)

# find include path
find_path(OMPL_INCLUDE_DIRS SpaceInformation.h
    PATHS ${OMPL_INCLUDE_PATH}
PATH_SUFFIXES base ompl/base include/ompl/base src/ompl/base)
if (OMPL_INCLUDE_DIRS)
    string(REGEX REPLACE "/ompl/base$" "" OMPL_INCLUDE_DIRS ${OMPL_INCLUDE_DIRS})
else()
    set(OMPL_INCLUDE_DIRS "")
endif()

find_package_handle_standard_args(OMPL DEFAULT_MSG OMPL_LIBRARIES OMPL_INCLUDE_DIRS)
