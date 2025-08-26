# VAMP Configuration Module
# ========================
# This module provides a clean, modular interface for VAMP integration
# with OMPL. It handles SIMD configuration, Python setup, and target management

include(CMakePackageConfigHelpers)

# Function to apply VAMP SIMD flags to a target
function(apply_vamp_simd_flags TARGET_NAME)
    if(NOT DEFINED VAMP_SIMD_FLAGS)
        return()
    endif()
    
    target_compile_options(${TARGET_NAME} PRIVATE ${VAMP_SIMD_FLAGS})
    message(STATUS "Applied VAMP SIMD flags to ${TARGET_NAME}")
endfunction()

# VAMP configuration function
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
    
    # Configure VAMP build options
    configure_vamp_build_options()
    
    # Setup SIMD environment
    setup_vamp_simd_environment()
    
    # Add VAMP subdirectory
    add_subdirectory(external/vamp)
    
    # Restore original flags
    restore_original_flags()
    
    # Configure VAMP targets
    configure_vamp_targets()
    
    set(OMPL_HAVE_VAMP TRUE CACHE BOOL "Whether VAMP integration is available" FORCE)
    message(STATUS "VAMP integration configured successfully")
endfunction()

# Configure VAMP build options
function(configure_vamp_build_options)
    # Pass VAMP build options to submodule
    set(VAMP_PORTABLE_BUILD ${VAMP_PORTABLE_BUILD} CACHE BOOL "Build VAMP with portable SIMD settings" FORCE)
    
    # Set VAMP Python bindings - use top-level option value
    set(VAMP_BUILD_PYTHON_BINDINGS ${VAMP_BUILD_PYTHON_BINDINGS} CACHE BOOL "Build VAMP Python bindings" FORCE)
    
    # Set VAMP-specific options
    set(VAMP_BUILD_CPP_DEMO OFF CACHE BOOL "Build VAMP C++ Demo Scripts" FORCE)
    set(VAMP_BUILD_OMPL_DEMO OFF CACHE BOOL "Build VAMP C++ OMPL Integration Demo Scripts" FORCE)
    set(VAMP_OMPL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "OMPL path for VAMP integration" FORCE)
endfunction()

# Setup VAMP SIMD environment
function(setup_vamp_simd_environment)
    # Store current flags to restore later
    set(OLD_CMAKE_C_FLAGS ${CMAKE_C_FLAGS} PARENT_SCOPE)
    set(OLD_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} PARENT_SCOPE)
    
    # Determine SIMD flags based on architecture and build mode
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        if(VAMP_PORTABLE_BUILD)
            set(SIMD_FLAGS_LIST "-march=x86-64" "-mavx2" "-mfma" "-mbmi2")
            message(STATUS "VAMP: Using portable SIMD flags for x86_64 (with BMI2)")
        else()
            set(SIMD_FLAGS_LIST "-march=native" "-mavx2")
            message(STATUS "VAMP: Using native SIMD flags for x86_64")
        endif()
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        if(VAMP_PORTABLE_BUILD)
            set(SIMD_FLAGS_LIST "-march=armv8-a")
            message(STATUS "VAMP: Using portable SIMD flags for ARM64")
        else()
            set(SIMD_FLAGS_LIST "-mcpu=native" "-mtune=native")
            message(STATUS "VAMP: Using native SIMD flags for ARM64")
        endif()
    else()
        message(FATAL_ERROR "Unsupported architecture ${CMAKE_SYSTEM_PROCESSOR} for VAMP")
    endif()
    
    # Convert list to space-separated string for CMAKE_C_FLAGS
    string(REPLACE ";" " " SIMD_FLAGS_STR "${SIMD_FLAGS_LIST}")
    
    # Apply SIMD flags globally for VAMP and its dependencies
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SIMD_FLAGS_STR}" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SIMD_FLAGS_STR}" CACHE STRING "" FORCE)
    
    # Store SIMD flags for later use
    set(VAMP_SIMD_FLAGS ${SIMD_FLAGS_LIST} PARENT_SCOPE)
endfunction()

# Restore original flags after VAMP configuration
function(restore_original_flags)
    if(DEFINED OLD_CMAKE_C_FLAGS)
        set(CMAKE_C_FLAGS ${OLD_CMAKE_C_FLAGS})
    endif()
    if(DEFINED OLD_CMAKE_CXX_FLAGS)
        set(CMAKE_CXX_FLAGS ${OLD_CMAKE_CXX_FLAGS})
    endif()
endfunction()

# Configure VAMP targets for OMPL integration
function(configure_vamp_targets)
    # Make VAMP C++ library available to OMPL
    if(TARGET vamp_cpp)
        # Set VAMP C++ library properties
        set_target_properties(vamp_cpp PROPERTIES
            POSITION_INDEPENDENT_CODE ON
            CXX_VISIBILITY_PRESET hidden
            VISIBILITY_INLINES_HIDDEN ON
        )
        
        # Apply SIMD flags to VAMP C++ library
        if(DEFINED VAMP_SIMD_FLAGS)
            target_compile_options(vamp_cpp INTERFACE ${VAMP_SIMD_FLAGS})
        endif()
        
        message(STATUS "VAMP C++ library target configured")
    endif()
    
    # Print VAMP build configuration
    if(VAMP_PORTABLE_BUILD)
        message(STATUS "VAMP: Using portable build mode for distribution")
    else()
        message(STATUS "VAMP: Using native build mode for best performance")
    endif()
    
endfunction()

# Function to apply VAMP SIMD flags to a target
function(apply_vamp_simd_flags TARGET_NAME)
    if(NOT DEFINED VAMP_SIMD_FLAGS)
        return()
    endif()
    
    target_compile_options(${TARGET_NAME} PRIVATE ${VAMP_SIMD_FLAGS})
    message(STATUS "Applied VAMP SIMD flags to ${TARGET_NAME}")
endfunction()

# Export VAMP configuration for other modules
set(VAMP_CONFIGURED TRUE)

# Make functions globally available by setting them as global properties
set_property(GLOBAL PROPERTY VAMP_FUNCTIONS_AVAILABLE TRUE) 