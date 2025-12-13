# VampConfig.cmake - VAMP Integration Module
function(configure_vamp)
    if(NOT OMPL_BUILD_VAMP)
        set(OMPL_HAVE_VAMP FALSE CACHE BOOL "Whether VAMP integration is available" FORCE)
        return()
    endif()

    # Check if VAMP submodule exists
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/vamp/CMakeLists.txt")
        message(WARNING "VAMP submodule not found. Run 'git submodule update --init --recursive' to initialize it.")
        set(OMPL_HAVE_VAMP FALSE CACHE BOOL "Whether VAMP integration is available" FORCE)
        return()
    endif()

    message(STATUS "Configuring VAMP integration...")
    
    # Pass build options to VAMP submodule
    set(VAMP_BUILD_PYTHON_BINDINGS ${VAMP_BUILD_PYTHON_BINDINGS} CACHE BOOL "Build VAMP Python bindings" FORCE)
    set(VAMP_BUILD_CPP_DEMO OFF CACHE BOOL "Build VAMP C++ Demo Scripts" FORCE)
    set(VAMP_BUILD_OMPL_DEMO OFF CACHE BOOL "Build VAMP C++ OMPL Integration Demo Scripts" FORCE)
    set(VAMP_OMPL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "OMPL path for VAMP integration" FORCE)
    
    # Add VAMP subdirectory - VAMP handles its own compiler settings and SIMD flags
    add_subdirectory(external/vamp)
    
    # Log build mode
    if(VAMP_PORTABLE_BUILD)
        message(STATUS "VAMP: Using portable build mode for distribution")
    else()
        message(STATUS "VAMP: Using native build mode for best performance")
    endif()
    
    if(VAMP_BUILD_PYTHON_BINDINGS)
        message(STATUS "VAMP: Python bindings enabled")
    else()
        message(STATUS "VAMP: Python bindings disabled (C++ only)")
    endif()
    
    set(OMPL_HAVE_VAMP TRUE CACHE BOOL "Whether VAMP integration is available" FORCE)
    message(STATUS "VAMP integration configured successfully")
endfunction()
