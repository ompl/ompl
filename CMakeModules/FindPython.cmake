# This code sets the following variables:
# PYTHON_FOUND         - boolean that indicates success
# PYTHON_EXEC          - path to python executable
# PYTHON_LIBRARIES     - path to the python library
# PYTHON_INCLUDE_DIRS  - path to where Python.h is found
# PYTHON_SITE_MODULES  - path to site-packages
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
#
# Finally, this module defines a number of macros:
# - find_boost_python(): Find the version of Boost.Python that matches the python interpreter
# - find_boost_numpy(): Find the version of Boost.Numpy that matches the python interpreter
# - install_python(PROGRAMS ...): Similar to install(PROGRAMS...), but replaces
#   "#!/usr/bin/env python" with "#!${PYTHON_EXEC}"

include(FindPackageHandleStandardArgs)

# allow specifying which Python installation to use
if (NOT PYTHON_EXEC)
    set(PYTHON_EXEC $ENV{PYTHON_EXEC})
endif (NOT PYTHON_EXEC)

if (NOT PYTHON_EXEC)
    find_program(PYTHON_EXEC
        NAMES "python${Python_FIND_VERSION}" "python3"
        PATHS
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.6\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.5\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.4\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\3.3\\InstallPath]
        [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\2.7\\InstallPath]
        DOC "Location of python executable to use")
endif(NOT PYTHON_EXEC)

# On macOS the python executable might be symlinked to the "real" location
# of the python executable. The header files and libraries are found relative
# to that path.
get_filename_component(PYTHON_EXEC_ "${PYTHON_EXEC}" REALPATH)
set(PYTHON_EXEC "${PYTHON_EXEC_}" CACHE FILEPATH "Path to Python interpreter")

string(REGEX REPLACE "/bin/python.*" "" PYTHON_PREFIX "${PYTHON_EXEC_}")
string(REGEX REPLACE "/bin/python.*" "" PYTHON_PREFIX2 "${PYTHON_EXEC}")

if(PYTHON_EXEC)
    execute_process(COMMAND "${PYTHON_EXEC}" "-c"
        "import sys; print('%d;%d;%d' % (sys.version_info[0],sys.version_info[1],sys.version_info[2]))"
        OUTPUT_VARIABLE PYTHON_VERSION_INFO
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    list(GET PYTHON_VERSION_INFO 0 PYTHON_VERSION_MAJOR)
    list(GET PYTHON_VERSION_INFO 1 PYTHON_VERSION_MINOR)
    list(GET PYTHON_VERSION_INFO 2 PYTHON_VERSION_MICRO)
    set(PYTHON_VERSION "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
    set(PYTHON_VERSION_NO_DOTS "${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
endif()

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
    OUTPUT_VARIABLE PYTHON_SITE_MODULES
    OUTPUT_STRIP_TRAILING_WHITESPACE)

function(find_python_module module)
    string(TOUPPER ${module} module_upper)
    set(_minversion "")
    if(ARGC GREATER 2)
        set(_minversion ${ARGV1})
        if (ARGV2 STREQUAL "REQUIRED")
            set(PY_${module}_FIND_REQUIRED ON)
        elseif (ARGV2 STREQUAL "QUIET")
            set(PY_${module}_FIND_QUIETLY ON)
        endif()
    elseif (ARGC GREATER 1)
        if (ARGV1 STREQUAL "REQUIRED")
            set(PY_${module}_FIND_REQUIRED ON)
        elseif (ARGV1 STREQUAL "QUIET")
            set(PY_${module}_FIND_QUIETLY ON)
        else()
            set(_minversion ${ARGV1})
        endif()
    endif()
    if(PYTHON_EXEC AND NOT PY_${module_upper})
        # A module's location is usually a directory, but for binary modules
        # it's a .so file.
        if (_minversion STREQUAL "")
            execute_process(COMMAND "${PYTHON_EXEC}" "-c"
                "import re, inspect, ${module}; print(re.compile('/__init__.py.*').sub('',inspect.getfile(${module})))"
                RESULT_VARIABLE _status OUTPUT_VARIABLE _location
                ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
            if(NOT _status)
                set(PY_${module_upper} ${_location} CACHE STRING
                    "Location of Python module ${module}")
            endif(NOT _status)
        else (_minversion STREQUAL "")
            execute_process(COMMAND "${PYTHON_EXEC}" "-c"
                "import re, inspect, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__version__+';'+inspect.getfile(${module})))"
                RESULT_VARIABLE _status
                OUTPUT_VARIABLE _verloc
                ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
            if(NOT _status)
                list(LENGTH _verloc _verloclength)
                if(_verloclength GREATER 1)
                    list(GET _verloc 1 _location)
                    list(GET _verloc 0 _version)
                else()
                    set(_version "0")
                endif()
                # get rid of version prefixes and suffixes so that
                # "v1.0rc2" becomes "1.0"
                string(REGEX MATCH "[0-9.]+" _version "${_version}")
                if (NOT ${_version} VERSION_LESS ${_minversion})
                    set(PY_${module_upper} ${_location} CACHE STRING
                        "Location of Python module ${module}")
                    set(PY_${module_upper}_VERSION ${_version} CACHE STRING
                        "Version of Python module ${module}")
                else()
                    message(WARNING "Module '${module}' version ${_version} found, but minimum version ${_minversion} required.")
                endif()
            endif(NOT _status)
        endif (_minversion STREQUAL "")
    endif(PYTHON_EXEC AND NOT PY_${module_upper})
    find_package_handle_standard_args(PY_${module} DEFAULT_MSG PY_${module_upper})
    if(PY_${module_upper})
        set_property(GLOBAL APPEND PROPERTY PY_MODULES_FOUND "${module}")
    else()
        set_property(GLOBAL APPEND PROPERTY PY_MODULES_NOTFOUND "${module}")
    endif()
endfunction(find_python_module)

# macro to attempt to find the *correct* Boost.Python library (i.e., the
# one that matches the version number of the python interpreter that was
# found).
macro(find_boost_python)
    if (PYTHON_FOUND)
        foreach(_bp_libname
            "python-py${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}"
            "python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}"
            "python${PYTHON_VERSION_MAJOR}" "python")
            string(TOUPPER ${_bp_libname} _bp_upper)
            set(_Boost_${_bp_upper}_HEADERS "boost/python.hpp")
            find_package(Boost COMPONENTS ${_bp_libname} QUIET)
            set(_bplib "${Boost_${_bp_upper}_LIBRARY}")
            if (_bplib)
                set(Boost_PYTHON_LIBRARY "${_bplib}")
                break()
            endif()
        endforeach()
    endif()
endmacro(find_boost_python)

# macro to attempt to find the *correct* Boost.Numpy library (i.e., the
# one that matches the version number of the python interpreter that was
# found).
macro(find_boost_numpy)
    if (PYTHON_FOUND)
        foreach(_bn_libname
            "numpy${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}"
            "numpy${PYTHON_VERSION_MAJOR}" "numpy")
            string(TOUPPER ${_bn_libname} _bn_upper)
            set(_Boost_${_bn_upper}_HEADERS "boost/numpy.hpp")
            find_package(Boost COMPONENTS ${_bn_libname} QUIET)
            set(_bnlib "${Boost_${_bn_upper}_LIBRARY}")
            if (_bnlib)
                set(Boost_NUMPY_LIBRARY "${_bnlib}")
                break()
            endif()
        endforeach()
    endif()
endmacro(find_boost_numpy)

# macro that is similar to install, but corrects the python interpreter
macro(install_python)
    if (PYTHON_FOUND)
        cmake_parse_arguments(install_python "" "DESTINATION;COMPONENT;RENAME" "PROGRAMS" "${ARGN}")
        foreach(script ${install_python_PROGRAMS})
            file(READ ${script} _contents)
            string(REPLACE "#!/usr/bin/env python" "#!${PYTHON_EXEC}" _fixed "${_contents}")
            get_filename_component(_realscript "${script}" NAME)
            file(WRITE "${PROJECT_BINARY_DIR}/${install_python_DESTINATION}/${_realscript}" "${_fixed}")
            install(PROGRAMS "${PROJECT_BINARY_DIR}/${install_python_DESTINATION}/${_realscript}"
                DESTINATION "${install_python_DESTINATION}"
                COMPONENT "${install_python_COMPONENT}"
                RENAME "${install_python_RENAME}")
        endforeach()
    endif()
endmacro(install_python)

find_package_handle_standard_args(Python DEFAULT_MSG
    PYTHON_LIBRARIES PYTHON_INCLUDE_DIRS PYTHON_SITE_MODULES)
