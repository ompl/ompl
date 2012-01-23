# - Find OPENDE

# This module finds if Open Dynamics Engine (OPENDE) is
# installed and determines where the include files and libraries
# are. The OPENDE_PATH variable (or environment variable) can be set
# to specify where to look for OPENDE. In addition, OPENDE_LIB_PATH
# and OPENDE_INCLUDE_PATH can be set to specify locations for the ode
# library and include files as well.
#
# The following variables are set:
#  OPENDE_LIBRARY     = location of OPENDE library
#  OPENDE_INCLUDE_DIR = include path for OPENDE
#  OPENDE_DEFINITIONS = defines for OPENDE; such as -DdSINGLE
#  OPENDE_VERSION     = the version of OPENDE that was found; empty if ode-config was not found

include(FindPackageHandleStandardArgs)

# Uncomment the following line to enable debug output
#set(_OPENDE_DEBUG_OUTPUT true)

if (NOT OPENDE_PATH)
  set(OPENDE_PATH $ENV{OPENDE_PATH})
endif()

if (NOT OPENDE_LIB_PATH)
  set(OPENDE_LIB_PATH $ENV{OPENDE_LIB_PATH})
endif()

if (NOT OPENDE_INCLUDE_PATH)
  set(OPENDE_INCLUDE_PATH $ENV{OPENDE_INCLUDE_PATH})
endif()

if (NOT OPENDE_LIB_PATH)
  set(OPENDE_LIB_PATH ${OPENDE_PATH})
endif()

if (NOT OPENDE_INCLUDE_PATH)
  set(OPENDE_INCLUDE_PATH ${OPENDE_PATH})
endif()

if (OPENDE_INCLUDE_PATH)
  set(_OPENDE_INCLUDE_HINTS ${OPENDE_INCLUDE_PATH})
else()
  set(_OPENDE_INCLUDE_HINTS "")
endif()

if (OPENDE_LIB_PATH)
  set(_OPENDE_LIB_HINTS ${OPENDE_LIB_PATH})
else()
  set(_OPENDE_LIB_HINTS "")
endif()


find_program(_OPENDE_CONFIG NAMES ode-config PATHS ${OPENDE_PATH} PATH_SUFFIXES bin)
if (_OPENDE_CONFIG)

  execute_process(COMMAND ${_OPENDE_CONFIG} --version OUTPUT_VARIABLE OPENDE_VERSION)
  # remove new line chars
  if (OPENDE_VERSION)
    string(REGEX REPLACE "[\r\n]+" "" OPENDE_VERSION ${OPENDE_VERSION})
  endif()

  execute_process(COMMAND ${_OPENDE_CONFIG} --cflags  OUTPUT_VARIABLE OPENDE_CFLAGS)
  if (OPENDE_CFLAGS)
    # split the flags
    string(REGEX MATCHALL "[^ ]+" OPENDE_CFLAGS_LIST ${OPENDE_CFLAGS})

    # get the defines and include hints only
    foreach(OPENDE_CFLAG ${OPENDE_CFLAGS_LIST})
      string(REGEX MATCH "^(-D[a-zA-Z0-9]+)[ \n\r]*$" MATCHED_FLAG ${OPENDE_CFLAG})
      if (MATCHED_FLAG)
    if (OPENDE_DEFINITIONS)
      set(OPENDE_DEFINITIONS ${OPENDE_DEFINITIONS} ${CMAKE_MATCH_1})
    else()
      set(OPENDE_DEFINITIONS ${CMAKE_MATCH_1})
    endif()
      else()
    string(REGEX MATCH "^-I([^ \r\n]+)[ \n\r]*$" MATCHED_FLAG ${OPENDE_CFLAG})
    if (MATCHED_FLAG)
      set(_OPENDE_INCLUDE_HINTS ${_OPENDE_INCLUDE_HINTS} ${CMAKE_MATCH_1})
    endif()
      endif()
    endforeach()
  endif()

  execute_process(COMMAND ${_OPENDE_CONFIG} --libs OUTPUT_VARIABLE OPENDE_LFLAGS)
  if (OPENDE_LFLAGS)
    string(REGEX MATCHALL "[^ ]+" OPENDE_LFLAGS_LIST ${OPENDE_LFLAGS})
    foreach(OPENDE_LFLAG ${OPENDE_LFLAGS_LIST})
      string(REGEX MATCH "^-L([^ \r\n]+)[ \n\r]*$" MATCHED_FLAG ${OPENDE_LFLAG})
      if (MATCHED_FLAG)
    set(_OPENDE_LIB_HINTS ${_OPENDE_LIB_HINTS} ${CMAKE_MATCH_1})
      endif()
    endforeach()
  endif()

endif()

# find the include path
find_path(OPENDE_INCLUDE_DIR ode.h PATHS ${_OPENDE_INCLUDE_HINTS} PATH_SUFFIXES ode include/ode)
if (OPENDE_INCLUDE_DIR)
  string(REGEX REPLACE "/ode$" "" OPENDE_INCLUDE_DIR ${OPENDE_INCLUDE_DIR})
endif()

# find the lib
find_library(OPENDE_LIBRARY ode PATHS ${_OPENDE_LIB_HINTS} PATH_SUFFIXES lib ode/src/.libs src/.libs)


if (_OPENDE_DEBUG_OUTPUT)
  message(STATUS "------- FindOPENDE.cmake Debug -------")
  message(STATUS "OPENDE_CONFIG = '${_OPENDE_CONFIG}'")
  message(STATUS "OPENDE_DEFINITIONS = '${OPENDE_DEFINITIONS}'")
  message(STATUS "_OPENDE_INCLUDE_HINTS = '${_OPENDE_INCLUDE_HINTS}'")
  message(STATUS "OPENDE_INCLUDE_DIR = '${OPENDE_INCLUDE_DIR}'")
  message(STATUS "_OPENDE_LIB_HINTS = '${_OPENDE_LIB_HINTS}'")
  message(STATUS "OPENDE_LIBRARY = '${OPENDE_LIBRARY}'")
  message(STATUS "OPENDE_VERSION = '${OPENDE_VERSION}'")
  message(STATUS "-----------------------------------")
endif()

find_package_handle_standard_args(OpenDE DEFAULT_MSG OPENDE_LIBRARY OPENDE_INCLUDE_DIR)
mark_as_advanced(_OPENDE_CONFIG _OPENDE_INCLUDE_H)
mark_as_advanced(OPENDE_LIBRARY OPENDE_INCLUDE OPENDE_DEFINITIONS OPENDE_VERSION)
