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

# look for boost AFTER find_boost_python which also uses
# find_package(Boost) internally and can overwrite a selected Boost library
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
set_package_properties(Eigen3 PROPERTIES
    URL "http://eigen.tuxfamily.org"
    PURPOSE "A linear algebra library used throughout OMPL.")
find_package(Eigen3 REQUIRED)

set_package_properties(yaml-cpp PROPERTIES
    URL "https://github.com/jbeder/yaml-cpp"
    PURPOSE "Used for parsing YAML configuration files, required for VAMP demos.")
find_package(yaml-cpp CONFIG REQUIRED)

# yaml-cpp might not come with alias or namespaced target names, so add it here.
# https://github.com/jbeder/yaml-cpp/issues/1025
# https://github.com/jbeder/yaml-cpp/pull/1196
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
    # Aliasing local imported targets is only supported in CMake >=3.18.
    # Promoting yaml-cpp to the global scope here prevents errors on
    # older CMake versions.
    # (see: https://cmake.org/cmake/help/latest/command/add_library.html#alias-libraries)
    set_target_properties(yaml-cpp PROPERTIES IMPORTED_GLOBAL TRUE)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

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

# default to building shared libs except Windows
if (MSVC)
    option(OMPL_BUILD_SHARED "Build OMPL as a shared library" OFF)
else()
    option(OMPL_BUILD_SHARED "Build OMPL as a shared library" ON)
endif()
add_subdirectory(src)
add_subdirectory(py-bindings)

# VAMP build options
option(VAMP_PORTABLE_BUILD "Build VAMP with portable SIMD settings for distribution (package maintainers)" OFF)
option(VAMP_BUILD_PYTHON_BINDINGS "Build VAMP Python bindings (requires OMPL_BUILD_VAMP=ON)" OFF)

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

# VAMP submodule integration
option(OMPL_BUILD_VAMP "Build VAMP submodule" ON)
if(OMPL_BUILD_VAMP)
    include(CMakeModules/VampConfig.cmake)
    configure_vamp()
    set(OMPL_HAVE_VAMP TRUE CACHE BOOL "Whether VAMP integration is available" FORCE)
    message(STATUS "VAMP integration enabled via OMPL_BUILD_VAMP=ON")
else()
    set(OMPL_HAVE_VAMP FALSE CACHE BOOL "Whether VAMP integration is available" FORCE)
    message(STATUS "VAMP integration disabled. Use -DOMPL_BUILD_VAMP=ON to enable.")
endif()

add_subdirectory(tests)

add_subdirectory(demos)
add_subdirectory(scripts)
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
    PURPOSE "VAMP (Vector Accelerated Motion Planning)")
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
