# - Find ODE

# This module finds if Open Dynamics Engine (ODE) is
# installed and determines where the include files and libraries
# are. The ODE_PATH variable (or environment variable) can be set
# to specify where to look for ODE. In addition, ODE_LIB_PATH
# and ODE_INCLUDE_PATH can be set to specify locations for the ode
# library and include files as well.
#
# The following variables are set:
#  ODE_LIBRARY     = location of ODE library
#  ODE_INCLUDE_DIR = include path for ODE
#  ODE_DEFINITIONS = defines for ODE; such as -DdSINGLE
#  ODE_VERSION     = the version of ODE that was found; empty if ode-config was not found

include(FindPackageHandleStandardArgs)

# Uncomment the following line to enable debug output
#set(_ODE_DEBUG_OUTPUT true)

if (NOT ODE_PATH)
  set(ODE_PATH $ENV{ODE_PATH})
endif()

if (NOT ODE_LIB_PATH)
  set(ODE_LIB_PATH $ENV{ODE_LIB_PATH})
endif()

if (NOT ODE_INCLUDE_PATH)
  set(ODE_INCLUDE_PATH $ENV{ODE_INCLUDE_PATH})
endif()

if (NOT ODE_LIB_PATH)
  set(ODE_LIB_PATH ${ODE_PATH})
endif()

if (NOT ODE_INCLUDE_PATH)
  set(ODE_INCLUDE_PATH ${ODE_PATH})
endif()

if (ODE_INCLUDE_PATH)
  set(_ODE_INCLUDE_HINTS ${ODE_INCLUDE_PATH})
else()
  set(_ODE_INCLUDE_HINTS "")
endif()

if (ODE_LIB_PATH)
  set(_ODE_LIB_HINTS ${ODE_LIB_PATH})
else()
  set(_ODE_LIB_HINTS "")
endif()


find_program(_ODE_CONFIG NAMES ode-config PATHS ${ODE_PATH} PATH_SUFFIXES bin)
if (_ODE_CONFIG)

  execute_process(COMMAND ${_ODE_CONFIG} --version OUTPUT_VARIABLE ODE_VERSION)
  # remove new line chars
  if (ODE_VERSION)
    string(REGEX REPLACE "[\r\n]+" "" ODE_VERSION ${ODE_VERSION})
  endif()

  execute_process(COMMAND ${_ODE_CONFIG} --cflags  OUTPUT_VARIABLE ODE_CFLAGS)
  if (ODE_CFLAGS)
    # split the flags
    string(REGEX MATCHALL "[^ ]+" ODE_CFLAGS_LIST ${ODE_CFLAGS})

    # get the defines and include hints only
    foreach(ODE_CFLAG ${ODE_CFLAGS_LIST})
      string(REGEX MATCH "^(-D[a-zA-Z0-9]+)[ \n\r]*$" MATCHED_FLAG ${ODE_CFLAG})
      if (MATCHED_FLAG)
    if (ODE_DEFINITIONS)
      set(ODE_DEFINITIONS ${ODE_DEFINITIONS} ${CMAKE_MATCH_1})
    else()
      set(ODE_DEFINITIONS ${CMAKE_MATCH_1})
    endif()
      else()
    string(REGEX MATCH "^-I([^ \r\n]+)[ \n\r]*$" MATCHED_FLAG ${ODE_CFLAG})
    if (MATCHED_FLAG)
      set(_ODE_INCLUDE_HINTS ${_ODE_INCLUDE_HINTS} ${CMAKE_MATCH_1})
    endif()
      endif()
    endforeach()
  endif()

  execute_process(COMMAND ${_ODE_CONFIG} --libs OUTPUT_VARIABLE ODE_LFLAGS)
  if (ODE_LFLAGS)
    string(REGEX MATCHALL "[^ ]+" ODE_LFLAGS_LIST ${ODE_LFLAGS})
    foreach(ODE_LFLAG ${ODE_LFLAGS_LIST})
      string(REGEX MATCH "^-L([^ \r\n]+)[ \n\r]*$" MATCHED_FLAG ${ODE_LFLAG})
      if (MATCHED_FLAG)
    set(_ODE_LIB_HINTS ${_ODE_LIB_HINTS} ${CMAKE_MATCH_1})
      endif()
    endforeach()
  endif()

endif()

# find the include path
find_path(ODE_INCLUDE_DIR ode.h PATHS ${_ODE_INCLUDE_HINTS} PATH_SUFFIXES ode include/ode)
if (ODE_INCLUDE_DIR)
  string(REGEX REPLACE "/ode$" "" ODE_INCLUDE_DIR ${ODE_INCLUDE_DIR})
endif()

# find the lib
find_library(ODE_LIBRARY ode PATHS ${_ODE_LIB_HINTS} PATH_SUFFIXES lib ode/src/.libs src/.libs)


if (_ODE_DEBUG_OUTPUT)
  message(STATUS "------- FindODE.cmake Debug -------")
  message(STATUS "ODE_CONFIG = '${_ODE_CONFIG}'")
  message(STATUS "ODE_DEFINITIONS = '${ODE_DEFINITIONS}'")
  message(STATUS "_ODE_INCLUDE_HINTS = '${_ODE_INCLUDE_HINTS}'")
  message(STATUS "ODE_INCLUDE_DIR = '${ODE_INCLUDE_DIR}'")
  message(STATUS "_ODE_LIB_HINTS = '${_ODE_LIB_HINTS}'")
  message(STATUS "ODE_LIBRARY = '${ODE_LIBRARY}'")
  message(STATUS "ODE_VERSION = '${ODE_VERSION}'")
  message(STATUS "-----------------------------------")
endif()

find_package_handle_standard_args(ODE DEFAULT_MSG ODE_LIBRARY ODE_INCLUDE_DIR)
mark_as_advanced(_ODE_CONFIG _ODE_INCLUDE_H)
mark_as_advanced(ODE_LIBRARY ODE_INCLUDE ODE_DEFINITIONS ODE_VERSION)
