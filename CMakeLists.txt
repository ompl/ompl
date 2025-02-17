cmake_minimum_required(VERSION 3.12)
project(ompl VERSION 1.6.0 LANGUAGES CXX)
set(OMPL_ABI_VERSION 17)

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

set_package_properties(Threads PROPERTIES
    URL "https://en.wikipedia.org/wiki/POSIX_Threads"
    PURPOSE "Pthreads is sometimes needed, depending on OS / compiler.")
find_package(Threads QUIET)

enable_testing()


set_package_properties(Boost PROPERTIES
    URL "https://www.boost.org"
    PURPOSE "Used throughout OMPL for data serialization, graphs, etc.")
set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.68 REQUIRED COMPONENTS serialization filesystem system program_options)

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

find_program(R_EXEC R)

add_subdirectory(src)
add_subdirectory(bindings)
# add_subdirectory(tests)
# add_subdirectory(demos)
# add_subdirectory(scripts)
add_subdirectory(doc)

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

# install catkin package.xml (needed for ROS installation)
install(FILES package.xml
    DESTINATION share/${PROJECT_NAME})

# Allows Colcon to find non-Ament packages when using workspace underlays
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

set_package_properties(PkgConfig PROPERTIES
    URL "https://www.freedesktop.org/wiki/Software/pkg-config/"
    PURPOSE "Used to find (compilation flags for) dependencies.")

feature_summary(DESCRIPTION INCLUDE_QUIET_PACKAGES WHAT ALL)
# additional feature info: show which Python modules were found and weren't found
get_property(PY_MODULES_FOUND GLOBAL PROPERTY PY_MODULES_FOUND)


# Create targets for building docker images
# See CMakeModules/OMPLUtils.cmake and scripts/docker for details
add_docker_target(ompl)
add_docker_target(plannerarena scripts)
