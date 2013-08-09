# - Find MORSE

# This module finds if the Modular OpenRobots Simulation Engine (MORSE)
# is installed and determines where the include files and libraries
# are. The MORSE_PATH variable (or environment variable) can be set
# to specify where to look for MORSE.
#
# The following variables are set:
#  MORSE_VERSION     = the version of MORSE that was found; empty if morse was not found

include(FindPackageHandleStandardArgs)

# Uncomment the following line to enable debug output
#set(_MORSE_DEBUG_OUTPUT true)

if (NOT MORSE_PATH)
  set(MORSE_PATH $ENV{MORSE_PATH})
endif()

find_program(MORSE_EXEC NAMES morse PATHS ${MORSE_PATH} PATH_SUFFIXES bin)
if (MORSE_EXEC)

  # MORSE's version info is written to stderr, not stdout
  execute_process(COMMAND ${MORSE_EXEC} --version ERROR_VARIABLE MORSE_VERSION)
  if (MORSE_VERSION)
    # remove newlines and 'morse '
    string(REGEX REPLACE "[\r\n]+" "" MORSE_VERSION ${MORSE_VERSION})
    string(REPLACE "morse " "" MORSE_VERSION ${MORSE_VERSION})
  endif()

endif()

if (_MORSE_DEBUG_OUTPUT)
  message(STATUS "------- FindMORSE.cmake Debug -------")
  message(STATUS "MORSE = '${MORSE_EXEC}'")
  message(STATUS "MORSE_VERSION = '${MORSE_VERSION}'")
  message(STATUS "-------------------------------------")
endif()

find_package_handle_standard_args(MORSE DEFAULT_MSG MORSE_EXEC)
mark_as_advanced(MORSE_EXEC MORSE_VERSION)

