# FindVamp.cmake
# Locates the VAMP library
#
# This module defines the following variables:
#   VAMP_FOUND - True if VAMP is found
#   VAMP_INCLUDE_DIRS - VAMP include directories
#   VAMP_LIBRARIES - VAMP libraries
#   VAMP_TARGETS - VAMP CMake targets

# Check if VAMP was built as part of OMPL
if(OMPL_HAVE_VAMP)
    set(VAMP_FOUND TRUE)
    
    # VAMP is built as a Python extension module
    # The main target is _core_ext
    if(TARGET _core_ext)
        set(VAMP_TARGETS _core_ext)
        set(VAMP_LIBRARIES _core_ext)
        
        # Get include directories from the target
        get_target_property(VAMP_INCLUDE_DIRS _core_ext INTERFACE_INCLUDE_DIRECTORIES)
        # Patch: Convert any relative paths to absolute paths
        set(_fixed_vamp_include_dirs "")
        foreach(dir ${VAMP_INCLUDE_DIRS})
            if(IS_ABSOLUTE "${dir}")
                list(APPEND _fixed_vamp_include_dirs "${dir}")
            else()
                list(APPEND _fixed_vamp_include_dirs "${CMAKE_CURRENT_SOURCE_DIR}/external/vamp/${dir}")
            endif()
        endforeach()
        set(VAMP_INCLUDE_DIRS ${_fixed_vamp_include_dirs})
        
        message(STATUS "VAMP found: built as part of OMPL")
    else()
        set(VAMP_FOUND FALSE)
        message(WARNING "VAMP target _core_ext not found despite OMPL_HAVE_VAMP being TRUE")
    endif()
else()
    # Try to find VAMP as an external installation
    find_path(VAMP_INCLUDE_DIR vamp
        PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
        /sw/include
        ${VAMP_ROOT}/include
        $ENV{VAMP_ROOT}/include
    )
    
    find_library(VAMP_LIBRARY
        NAMES vamp vamp_core
        PATHS
        /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        ${VAMP_ROOT}/lib
        $ENV{VAMP_ROOT}/lib
    )
    
    if(VAMP_INCLUDE_DIR AND VAMP_LIBRARY)
        set(VAMP_FOUND TRUE)
        set(VAMP_INCLUDE_DIRS ${VAMP_INCLUDE_DIR})
        set(VAMP_LIBRARIES ${VAMP_LIBRARY})
        message(STATUS "VAMP found: external installation")
    else()
        set(VAMP_FOUND FALSE)
        message(STATUS "VAMP not found")
    endif()
endif()

# Handle REQUIRED and QUIET arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Vamp
    REQUIRED_VARS VAMP_FOUND
    FOUND_VAR VAMP_FOUND
)

# Create imported target if VAMP is found externally
if(VAMP_FOUND AND NOT TARGET vamp::vamp)
    add_library(vamp::vamp UNKNOWN IMPORTED)
    set_target_properties(vamp::vamp PROPERTIES
        IMPORTED_LOCATION "${VAMP_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${VAMP_INCLUDE_DIRS}"
    )
endif()

# Mark as advanced
mark_as_advanced(VAMP_INCLUDE_DIR VAMP_LIBRARY) 