cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0017 NEW)
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
    cmake_policy(SET CMP0042 NEW)
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

# add "${prefix}/share/cmake/Modules", which is where Eigen3's module is in MacPorts.
if(${CMAKE_VERSION} VERSION_LESS 2.8.12)
    get_filename_component(CMAKE_ROOT_DIR "${CMAKE_ROOT}" PATH)
else()
    get_filename_component(CMAKE_ROOT_DIR "${CMAKE_ROOT}" DIRECTORY)
endif()
set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
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

# Add support in Boost::Python for std::shared_ptr
# This is a hack that replaces boost::shared_ptr related code with std::shared_ptr.
# Proper support for std::shared_ptr was added in Boost 1.63.
if(Boost_VERSION VERSION_LESS "106300")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/external")
endif()

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

find_package(Boost 1.54 COMPONENTS serialization filesystem system program_options unit_test_framework REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

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

# Eigen is needed for the InformedStateSampler
find_package(Eigen3 QUIET)
if (EIGEN3_FOUND)
    set(OMPL_HAVE_EIGEN3 1)
    include_directories("${EIGEN3_INCLUDE_DIR}")
endif()

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

# R is needed for running Planner Arena locally
find_program(R_EXEC R)

add_subdirectory(py-bindings)
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(demos)
add_subdirectory(scripts)
add_subdirectory(doc)

target_link_flags(ompl)
set(PKG_NAME "ompl")
set(PKG_DESC "The Open Motion Planning Library")
set(PKG_EXTERNAL_DEPS "${ompl_PKG_DEPS}")
set(PKG_OMPL_LIBS "-lompl ${ompl_LINK_FLAGS}")
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

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/ompl.conf"
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl${OMPL_INSTALL_SUFFIX}"
  COMPONENT ompl)

# script to install ompl on Ubuntu
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/install-ompl-ubuntu.sh.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/install-ompl-ubuntu.sh" @ONLY)

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
