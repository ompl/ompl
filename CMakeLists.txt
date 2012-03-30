cmake_minimum_required(VERSION 2.8)
project(ompl CXX C)

set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")
set(OMPL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(OMPL_DEMO_INSTALL_DIR share/ompl/demos
    CACHE STRING "Relative path to directory where demos will be installed")
set(OMPL_DOC_INSTALL_DIR share/ompl/doc
    CACHE STRING "Relative path to directory where documentation will be installed")

include_directories("${OMPL_INCLUDE_DIR}")

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
include(CompilerSettings)
find_package(Boost COMPONENTS date_time thread serialization filesystem system program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(Boost COMPONENTS odeint QUIET)
if (NOT ${Boost_VERSION} LESS 104400)
  option(OMPL_ODESOLVER "Enable OMPL ODE solver classes" ON)
  if(NOT Boost_ODEINT_FOUND)
    # Include bundled version of boost::odeint if it isn't installed natively
    set(ODEINT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/external")
    include_directories("${ODEINT_INCLUDE_DIR}")
  endif()
else()
  option(OMPL_ODESOLVER "Enable OMPL ODE solver classes" OFF)
endif()

if(MSVC)
  add_definitions(-DBOOST_ALL_NO_LIB)
  add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)
enable_testing()

# ODE is only needed for Open Dynamics Engine bindings
find_package(OpenDE QUIET)
set(OMPL_EXTENSION_OPENDE ${OPENDE_FOUND})
if (OPENDE_FOUND)
  add_definitions(${OPENDE_DEFINITIONS})
  include_directories(${OPENDE_INCLUDE_DIR})
endif()

include(OMPLVersion)
include(OMPLUtils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_subdirectory(src)
add_subdirectory(py-bindings)
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
  install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
  
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/doc/dox/FindOMPL.cmake"
    DESTINATION "share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules/")
endif()

# uninstall target
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

include(CPackSettings)
