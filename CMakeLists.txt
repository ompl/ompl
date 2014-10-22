cmake_minimum_required(VERSION 2.8)
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
    cmake_policy(SET CMP0042 OLD)
endif()
project(ompl CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    # By default, use Release mode
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)

    # On 32bit architectures, use RelWithDebInfo
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
    endif()
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
    set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
include(GNUInstallDirs)
include(CompilerSettings)
include(OMPLVersion)
include(OMPLUtils)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")

set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")
set(OMPL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(OMPL_DEMO_INSTALL_DIR "share/ompl${OMPL_INSTALL_SUFFIX}/demos"
    CACHE STRING "Relative path to directory where demos will be installed")

include_directories("${OMPL_INCLUDE_DIR}")


if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
    add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)
if(IS_ICPC)
    set(Boost_USE_STATIC_LIBS ON CACHE STRING "Use statically linked Boost libraries")
else(IS_ICPC)
    # Ensure dynamic linking with boost unit_test_framework
    add_definitions(-DBOOST_TEST_DYN_LINK)
endif(IS_ICPC)

# Do one quiet find_package on Boost to see if it is recent enough to
# have the try_join_for call
find_package(Boost QUIET 1.50)

# If Boost is not found at all, the check for version 1.50 below will die in
# the most ungraceful manner because Boost_VERSION is not defined.  Define the
# variable here for a more useful error.
if (NOT ${Boost_FOUND})
    set(Boost_VERSION 0)
endif()

# try_join_for requires the chrono library, so if we will use
# try_join_for, we need to include the chrono component
# Must recheck the Boost version, since update_bindings will re-run CMake
# and this will pass for versions of Boost < 1.50
if (${Boost_FOUND} AND ${Boost_VERSION} GREATER 104900) # Boost version is at least 1.50
  # we're using chrono
  find_package(Boost COMPONENTS date_time thread serialization filesystem system program_options unit_test_framework chrono REQUIRED)
else()
  # don't use chrono
  find_package(Boost COMPONENTS date_time thread serialization filesystem system program_options unit_test_framework REQUIRED)
endif()
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

if(${Boost_VERSION} LESS 105300)
  # Include bundled version of boost::odeint if it isn't installed natively
  set(ODEINT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/external")
  include_directories(SYSTEM "${ODEINT_INCLUDE_DIR}")
endif()

# on OS X we need to check whether to use libc++ or libstdc++ with clang++
if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
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


# pthread is sometimes needed, depending on OS / compiler
find_package(Threads QUIET)

enable_testing()

find_package(Python QUIET)
find_package(Boost COMPONENTS python QUIET)

# MORSE is only needed for Modular OpenRobots Simulation Engine bindings
find_package(MORSE QUIET)
set(OMPL_EXTENSION_MORSE ${MORSE_FOUND})

# OpenDE is only needed for Open Dynamics Engine bindings
find_package(OpenDE QUIET)
set(OMPL_EXTENSION_OPENDE ${OPENDE_FOUND})
if (OPENDE_FOUND)
  add_definitions(${OPENDE_DEFINITIONS})
  include_directories(SYSTEM ${OPENDE_INCLUDE_DIR})
endif()

find_package(Triangle QUIET)
set(OMPL_EXTENSION_TRIANGLE ${TRIANGLE_FOUND})
if (TRIANGLE_FOUND)
  include_directories(SYSTEM ${TRIANGLE_INCLUDE_DIR})
endif()

# If FLANN is installed, a wrapper for its nearest neighbor data structures can be used
find_package(flann 1.8.3 QUIET)
if (FLANN_FOUND)
    set(OMPL_HAVE_FLANN 1)
    include_directories(SYSTEM "${FLANN_INCLUDE_DIRS}")
endif()

option(OMPL_LOCAL_PYPLUSPLUS_INSTALL "Whether Py++ and dependencies should be installed in the build directory" OFF)
set(SUDO "sudo")
set(CMAKE_GCCXML_ARGS "")
if(OMPL_LOCAL_PYPLUSPLUS_INSTALL)
    set(SUDO "")
    set(CMAKE_GCCXML_ARGS "-DCMAKE_INSTALL_PREFIX=${PROJECT_BINARY_DIR}/pyplusplus")
    set(DISTUTILS_ARGS "--prefix=${PROJECT_BINARY_DIR}/pyplusplus")
endif()

add_subdirectory(py-bindings)
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(demos)
add_subdirectory(scripts)
add_subdirectory(doc)

if (NOT MSVC)
  set(PKG_NAME "ompl")
  set(PKG_DESC "The Open Motion Planning Library")
  set(PKG_OMPL_LIBS "-lompl")
  set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/ompl.pc")
  configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
  install(FILES "${pkg_conf_file}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/
    COMPONENT ompl
    RENAME "ompl${OMPL_INSTALL_SUFFIX}.pc")

  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/doc/markdown/FindOMPL.cmake"
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl${OMPL_INSTALL_SUFFIX}"
    COMPONENT ompl
    RENAME ompl-config.cmake)
endif()

# uninstall target
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

if (OMPL_VERSIONED_INSTALL)
  # script to create sym links
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/create_symlinks.sh.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/create_symlinks.sh" @ONLY)
  execute_process(COMMAND
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/create_symlinks.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/.symlinks/" DESTINATION .)
  # script to uninstall sym links
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/uninstall_symlinks.sh.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/uninstall_symlinks.sh" @ONLY)
  add_custom_target(uninstall_links COMMAND
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/uninstall_symlinks.sh")
  add_dependencies(uninstall uninstall_links)
endif()
include(CPackSettings)

option(OMPL_REGISTRATION "Enable one-time registration of OMPL" ON)
if (OMPL_REGISTRATION)
    find_file(OMPL_REGISTERED ".registered" PATHS "${CMAKE_CURRENT_SOURCE_DIR}" NO_DEFAULT_PATH)
    if (NOT OMPL_REGISTERED)
        file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/.registered" "")
        find_package(Python QUIET)
        if (PYTHON_FOUND)
            execute_process(COMMAND "${PYTHON_EXEC}" "-m" "webbrowser" "http://ompl.kavrakilab.org/core/register.html"
                OUTPUT_QUIET ERROR_QUIET)
        endif()
    endif()
endif()
