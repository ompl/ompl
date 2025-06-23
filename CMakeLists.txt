cmake_minimum_required(VERSION 3.12)
project(ompl VERSION 1.8.0 LANGUAGES CXX)
set(OMPL_ABI_VERSION 19)

# Use the FindBoost provided by CMake, rather than the one provided by Boost
# (for CMake >=3.30).
if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.30")
    cmake_policy(SET CMP0167 OLD)
endif()

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    # By default, use Release mode
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)

    # On 32bit architectures, use RelWithDebInfo
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
    endif()
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

list(APPEND CMAKE_MODULE_PATH
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
include(GNUInstallDirs)
include(FeatureSummary)
include(CompilerSettings)
include(OMPLUtils)

set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")
set(OMPL_DEMO_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/ompl/demos"
    CACHE STRING "Relative path to directory where demos will be installed")

if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
    add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)
# Ensure dynamic linking with boost unit_test_framework
add_definitions(-DBOOST_TEST_DYN_LINK)
# Avoid valgrind error due to overflow error, cf. https://github.com/ompl/ompl/issues/664
add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)

set_package_properties(Boost PROPERTIES
    URL "https://www.boost.org"
    PURPOSE "Used throughout OMPL for data serialization, graphs, etc.")
set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.68 REQUIRED COMPONENTS serialization program_options)

# on macOS we need to check whether to use libc++ or libstdc++ with clang++
if(CMAKE_CXX_COMPILER_ID MATCHES "^(Apple)?Clang$")
    include(GetPrerequisites)
    get_prerequisites("${Boost_SYSTEM_LIBRARY}" _libs 0 0 "/" "")
    set(CXXSTDLIB "")
    foreach(_lib ${_libs})
        if(_lib MATCHES "libc\\+\\+")
            set(CXXSTDLIB "libc++")
        elseif(_lib MATCHES "libstdc\\+\\+")
            set(CXXSTDLIB "libstdc++")
        endif()
    endforeach()
    if(CXXSTDLIB)
        add_definitions(-stdlib=${CXXSTDLIB})
    endif()
endif()

set_package_properties(Threads PROPERTIES
    URL "https://en.wikipedia.org/wiki/POSIX_Threads"
    PURPOSE "Pthreads is sometimes needed, depending on OS / compiler.")
find_package(Threads QUIET)

enable_testing()

set_package_properties(Python PROPERTIES
    URL "https://www.python.org"
    PURPOSE "Used for python bindings.")
find_package(Python QUIET)
find_boost_python()

if(PYTHON_FOUND)
    set_package_properties(pypy PROPERTIES
        URL "https://pypy.org"
        PURPOSE "Used to speed up the generation of python bindings.")
    find_package(pypy QUIET)
endif()

set_package_properties(Eigen3 PROPERTIES
    URL "http://eigen.tuxfamily.org"
    PURPOSE "A linear algebra library used throughout OMPL.")
find_package(Eigen3 REQUIRED)

set_package_properties(Triangle PROPERTIES
    URL "http://www.cs.cmu.edu/~quake/triangle.html"
    PURPOSE "Used to create triangular decompositions of polygonal 2D environments.")
find_package(Triangle QUIET)
set(OMPL_EXTENSION_TRIANGLE ${TRIANGLE_FOUND})

set_package_properties(flann PROPERTIES
    URL "https://github.com/mariusmuja/flann"
    PURPOSE "If detetected, FLANN can be used for nearest neighbor queries by OMPL.")
find_package(flann CONFIG 1.9.2 QUIET)
if (FLANN_FOUND)
    set(OMPL_HAVE_FLANN 1)
else ()
    set(OMPL_HAVE_FLANN 0)
endif()

set_package_properties(spot PROPERTIES
    URL "http://spot.lrde.epita.fr"
    PURPOSE "Used for constructing finite automata from LTL formulae.")
find_package(spot)
if (spot_FOUND)
    set(OMPL_HAVE_SPOT 1)
endif()

set_package_properties(Doxygen PROPERTIES
    URL "http://doxygen.org"
    PURPOSE "Used to create the OMPL documentation (i.e., https://ompl.kavrakilab.org).")
find_package(Doxygen)

# Numpy is used to convert Eigen matrices/vectors to numpy arrays
if(PYTHON_FOUND)
    find_python_module(numpy)
    if (PY_NUMPY)
        find_boost_numpy()
        if(Boost_NUMPY_LIBRARY)
            set(OMPL_HAVE_NUMPY 1)
            include_directories(SYSTEM "${PY_NUMPY}/core/include")
        endif()
    endif()
endif()

# R is needed for running Planner Arena locally
find_program(R_EXEC R)

add_subdirectory(src)
add_subdirectory(py-bindings)
add_subdirectory(tests)
add_subdirectory(demos)
add_subdirectory(scripts)
add_subdirectory(doc)


# VAMP submodule integration
option(OMPL_BUILD_VAMP "Build VAMP submodule" ON)
if(OMPL_BUILD_VAMP)
    # Check if vamp submodule is initialized
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/vamp/CMakeLists.txt")
        message(STATUS "Building VAMP submodule...")
        
        # Set up Python environment for VAMP BEFORE including VAMP
        # This ensures nanobind gets the correct Python configuration
        if(PYTHON_FOUND)
            message(STATUS "Setting up Python environment for VAMP/nanobind...")
            
            # Create the Python::Module target that nanobind expects
            if(NOT TARGET Python::Module)
                add_library(Python::Module INTERFACE IMPORTED)
                if(PYTHON_INCLUDE_DIRS)
                    set_target_properties(Python::Module PROPERTIES
                        INTERFACE_INCLUDE_DIRECTORIES "${PYTHON_INCLUDE_DIRS}")
                endif()
                if(PYTHON_LIBRARIES)
                    set_target_properties(Python::Module PROPERTIES
                        INTERFACE_LINK_LIBRARIES "${PYTHON_LIBRARIES}")
                endif()
            endif()
            
            # Create Python::Interpreter target as well
            if(NOT TARGET Python::Interpreter)
                add_library(Python::Interpreter INTERFACE IMPORTED)
                if(PYTHON_INCLUDE_DIRS)
                    set_target_properties(Python::Interpreter PROPERTIES
                        INTERFACE_INCLUDE_DIRECTORIES "${PYTHON_INCLUDE_DIRS}")
                endif()
            endif()
            
            # Set up modern CMake Python variables
            set(Python_Interpreter_FOUND TRUE)
            set(Python_Development_FOUND TRUE)
            if(PYTHON_EXEC)
                set(Python_EXECUTABLE ${PYTHON_EXEC})
            endif()
            if(PYTHON_INCLUDE_DIRS)
                set(Python_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
            endif()
            if(PYTHON_LIBRARIES)
                set(Python_LIBRARIES ${PYTHON_LIBRARIES})
            endif()
            if(PYTHON_VERSION)
                set(Python_VERSION ${PYTHON_VERSION})
            endif()
            if(PYTHON_VERSION_MAJOR)
                set(Python_VERSION_MAJOR ${PYTHON_VERSION_MAJOR})
            endif()
            if(PYTHON_VERSION_MINOR)
                set(Python_VERSION_MINOR ${PYTHON_VERSION_MINOR})
            endif()
            
            message(STATUS "Python environment setup complete for VAMP/nanobind")
            
            # Set VAMP compiler flags before including VAMP
            if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
                # Store current flags
                set(OLD_CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
                set(OLD_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
                
                # Set VAMP-specific flags
                set(VAMP_ARCH "-march=native -mavx2")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${VAMP_ARCH}")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VAMP_ARCH}")
                
                message(STATUS "Setting VAMP AVX2 compiler flags: ${VAMP_ARCH}")
            endif()
            
            # Set vamp-specific options
            set(VAMP_BUILD_CPP_DEMO OFF CACHE BOOL "Build VAMP C++ Demo Scripts" FORCE)
            set(VAMP_BUILD_OMPL_DEMO ON CACHE BOOL "Build VAMP C++ OMPL Integration Demo Scripts" FORCE)
            set(VAMP_OMPL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "OMPL path for VAMP integration" FORCE)
            
            # Add vamp as subdirectory
            add_subdirectory(external/vamp)
            
            # Restore original flags after VAMP is configured
            if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
                set(CMAKE_C_FLAGS ${OLD_CMAKE_C_FLAGS})
                set(CMAKE_CXX_FLAGS ${OLD_CMAKE_CXX_FLAGS})
            endif()
            
            # Make vamp targets available to OMPL
            set(OMPL_HAVE_VAMP TRUE)
            message(STATUS "VAMP integration enabled")
        else()
            message(WARNING "Python not found. VAMP requires Python 3.8+ with development headers.")
            set(OMPL_BUILD_VAMP FALSE)
        endif()
    else()
        message(WARNING "VAMP submodule not found. Run 'git submodule update --init --recursive' to initialize it.")
        set(OMPL_HAVE_VAMP FALSE)
    endif()
else()
    set(OMPL_HAVE_VAMP FALSE)
    message(STATUS "VAMP build disabled")
endif()


if(NOT MSVC)
    target_link_flags(ompl)
    set(PKG_NAME "ompl")
    set(PKG_DESC "The Open Motion Planning Library")
    set(PKG_EXTERNAL_DEPS "Eigen3::Eigen ${ompl_PKG_DEPS}")
    set(PKG_OMPL_LIBS "-lompl ${ompl_LINK_FLAGS}")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/ompl.pc.in"
                   "${CMAKE_CURRENT_BINARY_DIR}/ompl.pc" @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ompl.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
        COMPONENT ompl)
endif()

set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
include(CMakePackageConfigHelpers)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
configure_package_config_file(omplConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/omplConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
    PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/omplConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/omplConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/omplConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
    COMPONENT ompl)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
    COMPONENT ompl
    FILES_MATCHING PATTERN "Find*.cmake")
install(TARGETS ompl
    EXPORT omplExport
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT ompl)
install(EXPORT omplExport
    NAMESPACE ompl::
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake)
export(EXPORT omplExport
    FILE "${CMAKE_CURRENT_BINARY_DIR}/omplExport.cmake")

# script to install ompl on Ubuntu
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/install-ompl-ubuntu.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/install-ompl-ubuntu.sh" @ONLY)

# uninstall target
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

include(CPackSettings)

option(OMPL_REGISTRATION "Enable one-time registration of OMPL" ON)
if (OMPL_REGISTRATION)
    find_file(OMPL_REGISTERED ".registered" PATHS "${CMAKE_CURRENT_SOURCE_DIR}" NO_DEFAULT_PATH)
    if (NOT OMPL_REGISTERED)
        file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/.registered" "")
        find_package(Python QUIET)
        if (PYTHON_FOUND)
            execute_process(COMMAND ${CMAKE_COMMAND} -E env
               TERM="" "${PYTHON_EXEC}" "-m" "webbrowser" "https://ompl.kavrakilab.org/core/register.html"
                OUTPUT_QUIET ERROR_QUIET)
        endif()
    endif()
endif()

# install catkin package.xml (needed for ROS installation)
install(FILES package.xml
    DESTINATION share/${PROJECT_NAME})

# Allows Colcon to find non-Ament packages when using workspace underlays
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

set_package_properties(PkgConfig PROPERTIES
    URL "https://www.freedesktop.org/wiki/Software/pkg-config/"
    PURPOSE "Used to find (compilation flags for) dependencies.")
set_package_properties(castxml PROPERTIES
    URL "https://github.com/CastXML/CastXML"
    PURPOSE "Used to generate Python bindings.")
    
# Add VAMP to feature summary
set_package_properties(VAMP PROPERTIES
    URL "https://github.com/KavrakiLab/vamp"
    PURPOSE "VAMP (Visualization and Analysis of Motion Planning) provides tools for motion planning visualization and analysis.")
if(OMPL_HAVE_VAMP)
    set(VAMP_FOUND TRUE)
else()
    set(VAMP_FOUND FALSE)
endif()

feature_summary(DESCRIPTION INCLUDE_QUIET_PACKAGES WHAT ALL)
# additional feature info: show which Python modules were found and weren't found
get_property(PY_MODULES_FOUND GLOBAL PROPERTY PY_MODULES_FOUND)
if(PY_MODULES_FOUND)
    list(REMOVE_DUPLICATES PY_MODULES_FOUND)
    string(REPLACE ";" " " PY_MODULES_FOUND_STR "${PY_MODULES_FOUND}")
    message(STATUS "The following Python modules were found:\n\n * ${PY_MODULES_FOUND_STR}\n")
endif()
get_property(PY_MODULES_NOTFOUND GLOBAL PROPERTY PY_MODULES_NOTFOUND)
if(PY_MODULES_NOTFOUND)
    list(REMOVE_DUPLICATES PY_MODULES_NOTFOUND)
    string(REPLACE ";" " " PY_MODULES_NOTFOUND_STR "${PY_MODULES_NOTFOUND}")
    message(STATUS "The following Python modules were NOT found:\n\n * ${PY_MODULES_NOTFOUND_STR}\n")
endif()

# Create targets for building docker images
# See CMakeModules/OMPLUtils.cmake and scripts/docker for details
add_docker_target(ompl)
add_docker_target(plannerarena scripts)
