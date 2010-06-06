find_program(PYTHON_EXEC "python${PYTHON_VERSION}" 
	DOC "Location of python executable to use")
string(REGEX REPLACE "/bin/python${PYTHON_VERSION}$" "" PYTHON_PREFIX "${PYTHON_EXEC}")
find_library(PYTHON_LIBRARIES "python${PYTHON_VERSION}" PATHS "${PYTHON_PREFIX}" 
	PATH_SUFFIXES "lib" "lib/python${PYTHON_VERSION}/config" 
	DOC "Python libraries" NO_DEFAULT_PATH)
find_path(PYTHON_INCLUDE_DIRS "Python.h"
	PATHS "${PYTHON_PREFIX}/include/python${PYTHON_VERSION}"
	DOC "Python include directories" NO_DEFAULT_PATH)
execute_process(COMMAND "${PYTHON_EXEC}" "-c" 
	"from distutils.sysconfig import get_python_lib; print get_python_lib()"
	OUTPUT_VARIABLE PYTHON_SITE_MODULES
	OUTPUT_STRIP_TRAILING_WHITESPACE)

function(find_python_module module)
	execute_process(COMMAND "${PYTHON_EXEC}" "-c" "import ${module}"
		RESULT_VARIABLE _${module}_status ERROR_QUIET)
	string(TOUPPER ${module} module_upper)
	if(NOT ${module_upper}_FOUND)
		if(_${module}_status)
			set(${module_upper}_FOUND FALSE)
			message(STATUS "Checking if Python module ${module} exists - no")
		else(_${module}_status)
			set(${module_upper}_FOUND TRUE)
			message(STATUS "Checking if Python module ${module} exists - yes")
		endif(_${module}_status)
		set(${module_upper}_FOUND ${${module_upper}_FOUND} CACHE BOOL
			"Whether Python module ${module} exists")
	endif(NOT ${module_upper}_FOUND)
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

