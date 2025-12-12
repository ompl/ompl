# FindNanobind.cmake
# This module finds nanobind and sets it up for use.
# It uses the standard CMake Python finder (not the custom FindPython.cmake)

include(FindPackageHandleStandardArgs)

# Save the current module path
set(_CMAKE_MODULE_PATH_SAVED ${CMAKE_MODULE_PATH})

# Temporarily remove CMakeModules from the path to use built-in Python finder
list(REMOVE_ITEM CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

# Use the standard CMake Python finder (required by nanobind)
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# Restore the module path
set(CMAKE_MODULE_PATH ${_CMAKE_MODULE_PATH_SAVED})

# Check if nanobind submodule exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/nanobind/CMakeLists.txt")
    # Add nanobind subdirectory
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/nanobind)
    set(NANOBIND_FOUND TRUE)
else()
    set(NANOBIND_FOUND FALSE)
    message(WARNING "Nanobind submodule not found at ${CMAKE_CURRENT_SOURCE_DIR}/external/nanobind")
endif()

find_package_handle_standard_args(Nanobind
    REQUIRED_VARS NANOBIND_FOUND
    VERSION_VAR nanobind_VERSION
)
