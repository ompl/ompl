cmake_minimum_required(VERSION 3.17)
project(ompl VERSION 1.8.0 LANGUAGES CXX)
set(OMPL_ABI_VERSION 18)

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

set_package_properties(Python PROPERTIES
    URL "https://www.python.org"
    PURPOSE "Used for python bindings.")
set(Python_FIND_STRATEGY VERSION)
find_package(Python 3.8 COMPONENTS Interpreter Development)

set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")
set(OMPL_DEMO_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/ompl/demos"
    CACHE STRING "Relative path to directory where demos will be installed")
set(OMPL_PYTHON_INSTALL_DIR "${Python_SITELIB}"
    CACHE FILEPATH "Path to directory where the OMPL Python bindings will be installed")

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
find_package(Boost 1.68 REQUIRED COMPONENTS serialization filesystem system program_options)

enable_testing()


if(TARGET Python::Module)
    include(FetchContent)
    FetchContent_Declare(
        nanobind
        GIT_REPOSITORY https://github.com/wjakob/nanobind.git
        GIT_TAG        v2.7.0
        GIT_SHALLOW    ON
    )
    FetchContent_MakeAvailable(nanobind)
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

add_subdirectory(src)
if(TARGET Python::Module)
    add_subdirectory(bindings)
endif()
add_subdirectory(tests)
add_subdirectory(demos)
add_subdirectory(scripts)
add_subdirectory(doc)

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
        if (TARGET Python::Module)
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

feature_summary(DESCRIPTION INCLUDE_QUIET_PACKAGES WHAT ALL)

# Create targets for building docker images
# See CMakeModules/OMPLUtils.cmake and scripts/docker for details
add_docker_target(ompl)
