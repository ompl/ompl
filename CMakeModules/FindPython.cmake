# This code sets the following variables:
# PYTHON_FOUND         - boolean that indicates success
# PYTHON_EXEC          - path to python executable
# PYTHON_LIBRARIES     - path to the python library
# PYTHON_INCLUDE_DIRS  - path to where Python.h is found
# PYTHON_SITE_MODULES  - path to site-packages
# PYTHON_ARCH          - name of architecture to be used for platform-specific
#                        binary modules
# PYTHON_VERSION       - version of python
# PYTHON_VERSION_MAJOR - major version number
# PYTHON_VERSION_MAJOR - minor version number
# PYTHON_VERSION_MICRO - micro version number
#
# You can optionally include the version number when using this package
# like so:
#   find_package(python 2.6)
#
# This code defines a helper function find_python_module(). It can be used
# like so:
#   find_python_module(numpy [version] [REQUIRED|QUIET])
# If numpy is found, the variable PY_NUMPY contains the location of the numpy
# module. If a particular minimum version or higher is required, use the
# version argument:
#   find_python_module(numpy 1.9.0)
# If the module is required add the keyword "REQUIRED":
#   find_python_module(numpy REQUIRED)
# If no output should be displayed about whether the module is found, use the
# QUIET argument:
#   find_python_module(numpy QUIET)

include(FindPackageHandleStandardArgs)

# allow specifying which Python installation to use
if (NOT PYTHON_EXEC)
    set(PYTHON_EXEC $ENV{PYTHON_EXEC})
endif (NOT PYTHON_EXEC)

if (NOT PYTHON_EXEC)
    find_program(PYTHON_EXEC "python${Python_FIND_VERSION}"
        PATHS
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.4\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.3\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.2\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.1\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.0\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\2.7\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\2.6\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\2.5\\InstallPath]
        DOC "Location of python executable to use")
endif(NOT PYTHON_EXEC)

# if Python is still not found, return
if (NOT PYTHON_EXEC)
    # dummy function
    function(find_python_module module)
        return()
    endfunction(find_python_module)
    return()
endif()

# On OS X the python executable might be symlinked to the "real" location
# of the python executable. The header files and libraries are found relative
# to that path.
# For CMake 2.6 and below, the REALPATH option is included in the ABSOLUTE option
if (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)
  get_filename_component(PYTHON_EXEC_ "${PYTHON_EXEC}" REALPATH)
else()
  get_filename_component(PYTHON_EXEC_ "${PYTHON_EXEC}" ABSOLUTE)
endif()
set(PYTHON_EXEC "${PYTHON_EXEC_}" CACHE FILEPATH "Path to Python interpreter")

string(REGEX REPLACE "/bin/python.*" "" PYTHON_PREFIX "${PYTHON_EXEC_}")
string(REGEX REPLACE "/bin/python.*" "" PYTHON_PREFIX2 "${PYTHON_EXEC}")

execute_process(COMMAND "${PYTHON_EXEC}" "-c"
    "import sys; print('%d;%d;%d' % (sys.version_info[0],sys.version_info[1],sys.version_info[2]))"
    OUTPUT_VARIABLE PYTHON_VERSION_INFO
    OUTPUT_STRIP_TRAILING_WHITESPACE)
list(GET PYTHON_VERSION_INFO 0 PYTHON_VERSION_MAJOR)
list(GET PYTHON_VERSION_INFO 1 PYTHON_VERSION_MINOR)
list(GET PYTHON_VERSION_INFO 2 PYTHON_VERSION_MICRO)
set(PYTHON_VERSION "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
set(PYTHON_VERSION_NO_DOTS "${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")

find_library(PYTHON_LIBRARIES
    NAMES "python${PYTHON_VERSION_NO_DOTS}" "python${PYTHON_VERSION}" "python${PYTHON_VERSION}m"
    PATHS
        "${PYTHON_PREFIX}/lib"
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION}\\InstallPath]/libs
    PATH_SUFFIXES "" "python${PYTHON_VERSION}/config" "x86_64-linux-gnu" "i386-linux-gnu"
    DOC "Python libraries" NO_DEFAULT_PATH)

find_path(PYTHON_INCLUDE_DIRS "Python.h"
    PATHS
        "${PYTHON_PREFIX}/include"
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${PYTHON_VERSION}\\InstallPath]/include
    PATH_SUFFIXES python${PYTHON_VERSION} python${PYTHON_VERSION}m
    DOC "Python include directories" NO_DEFAULT_PATH)

execute_process(COMMAND "${PYTHON_EXEC}" "-c"
    "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
    OUTPUT_VARIABLE PYTHON_SITE_MODULES_
    OUTPUT_STRIP_TRAILING_WHITESPACE)
string(REGEX REPLACE "^${PYTHON_PREFIX2}/" "" PYTHON_SITE_MODULES "${PYTHON_SITE_MODULES_}")

function(find_python_module module)
    string(TOUPPER ${module} module_upper)
    set(_minversion "")
    if(ARGC GREATER 2)
        set(_minversion ${ARGV1})
        if (ARGV2 STREQUAL "REQUIRED")
            set(PY_${module}_FIND_REQUIRED TRUE)
        elseif (ARGV2 STREQUAL "QUIET")
            set(PY_${module}_FIND_QUIETLY TRUE)
        endif()
    elseif (ARGC GREATER 1)
        if (ARGV1 STREQUAL "REQUIRED")
            set(PY_${module}_FIND_REQUIRED TRUE)
        elseif (ARGV1 STREQUAL "QUIET")
            set(PY_${module}_FIND_QUIETLY TRUE)
        else()
            set(_minversion ${ARGV1})
        endif()
    endif()
    if(NOT PY_${module_upper})
        # A module's location is usually a directory, but for binary modules
        # it's a .so file.
        if (_minversion STREQUAL "")
            execute_process(COMMAND "${PYTHON_EXEC}" "-c"
                "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
                RESULT_VARIABLE _status OUTPUT_VARIABLE _location
                ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
            if(NOT _status)
                set(PY_${module_upper} ${_location} CACHE STRING
                    "Location of Python module ${module}")
            endif(NOT _status)
        else (_minversion STREQUAL "")
            execute_process(COMMAND "${PYTHON_EXEC}" "-c"
                "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__version__+';'+${module}.__file__))"
                RESULT_VARIABLE _status
                OUTPUT_VARIABLE _verloc
                ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
            list(GET _verloc 1 _location)
            list(GET _verloc 0 _version)
            message("${_status} ${_verloc} ${_version}")
            if(NOT _status)
                if (NOT ${_version} VERSION_LESS ${_minversion})
                    set(PY_${module_upper} ${_location} CACHE STRING
                        "Location of Python module ${module}")
                    set(PY_${module_upper}_VERSION ${_version} CACHE STRING
                        "Version of Python module ${module}")
                else()
                    message(SEND_ERROR "Module '${module}' version ${_version} found, but minimum version ${_minversion} required.")
                endif()
            endif(NOT _status)
        endif (_minversion STREQUAL "")
    endif(NOT PY_${module_upper})
    find_package_handle_standard_args(PY_${module} DEFAULT_MSG PY_${module_upper})
endfunction(find_python_module)

set(PYTHON_ARCH "unknown")
if(APPLE)
    set(PYTHON_ARCH "darwin")
else(APPLE)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
            set(PYTHON_ARCH "linux2")
        else(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
            set(PYTHON_ARCH "linux")
        endif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    else(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            set(PYTHON_ARCH "windows")
        endif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    endif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
endif(APPLE)

find_package_handle_standard_args(Python DEFAULT_MSG
    PYTHON_LIBRARIES PYTHON_INCLUDE_DIRS PYTHON_SITE_MODULES)
