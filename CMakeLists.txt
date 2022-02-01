cmake_minimum_required(VERSION 3.5)
project(ompl VERSION 1.5.2 LANGUAGES CXX)
set(OMPL_ABI_VERSION 16)

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

set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
include(GNUInstallDirs)
include(FeatureSummary)
include(CompilerSettings)
include(OMPLUtils)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")
set(OMPL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_BINARY_DIR}/src")
set(OMPL_DEMO_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/ompl/demos"
    CACHE STRING "Relative path to directory where demos will be installed")

include_directories("${OMPL_INCLUDE_DIRS}")

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
set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# Add support in Boost::Python for std::shared_ptr
# This is a hack that replaces boost::shared_ptr related code with std::shared_ptr.
# Proper support for std::shared_ptr was added in Boost 1.63.
if (NOT Boost_VERSION_STRING)
    if (Boost_VERSION_MAJOR)
        set(Boost_VERSION_STRING
            "${Boost_VERSION_MAJOR}.${Boost_VERSION_MINOR}.${Boost_VERSION_PATCH}")
    else()
        set(Boost_VERSION_STRING
            "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_PATCH_VERSION}")
    endif()
endif()
if(Boost_VERSION_STRING VERSION_LESS "1.63.0")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/external")
endif()


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
include_directories("${EIGEN3_INCLUDE_DIR}")

set_package_properties(MORSE PROPERTIES
    URL "https://www.openrobots.org/wiki/morse"
    PURPOSE "OMPL includes a plugin for the MORSE Robot Simulation engine.")
find_package(MORSE QUIET)
set(OMPL_EXTENSION_MORSE ${MORSE_FOUND})

set_package_properties(ODE PROPERTIES
    URL "http://ode.org"
    PURPOSE "The Open Dynamics Engine can be used as a \"black box\" for kinodynamic planning.")
find_package(ODE QUIET)
set(OMPL_EXTENSION_ODE ${ODE_FOUND})
if (OMPL_EXTENSION_ODE)
  include_directories(${ODE_INCLUDE_DIRS})
endif()

set_package_properties(Triangle PROPERTIES
    URL "http://www.cs.cmu.edu/~quake/triangle.html"
    PURPOSE "Used to create triangular decompositions of polygonal 2D environments.")
find_package(Triangle QUIET)
set(OMPL_EXTENSION_TRIANGLE ${TRIANGLE_FOUND})
if (OMPL_EXTENSION_TRIANGLE)
  include_directories(${TRIANGLE_INCLUDE_DIR})
endif()

set_package_properties(flann PROPERTIES
    URL "https://github.com/mariusmuja/flann"
    PURPOSE "If detetected, FLANN can be used for nearest neighbor queries by OMPL.")
find_package(flann 1.8.3 QUIET)
if (FLANN_FOUND)
    set(OMPL_HAVE_FLANN 1)
    include_directories("${FLANN_INCLUDE_DIRS}")
    link_directories(${FLANN_LIBRARY_DIRS})
endif()

set_package_properties(spot PROPERTIES
    URL "http://spot.lrde.epita.fr"
    PURPOSE "Used for constructing finite automata from LTL formulae.")
find_package(spot)
if (spot_FOUND)
    set(OMPL_HAVE_SPOT 1)
    include_directories("${SPOT_INCLUDE_DIRS}")
    link_directories(${SPOT_LIBRARY_DIRS})
endif()

set_package_properties(Doxygen PROPERTIES
    URL "http://doxygen.org"
    PURPOSE "Used to create the OMPL documentation (i.e., https://ompl.kavrakilab.org).")
find_package(Doxygen)

# Numpy is used to convert Eigen matrices/vectors to numpy arrays
if(PYTHON_FOUND AND NOT Boost_VERSION_STRING VERSION_LESS "1.63.0")
    find_python_module(numpy)
    if (PY_NUMPY)
        find_boost_numpy()
        if(Boost_NUMPY_LIBRARY)
            set(OMPL_HAVE_NUMPY 1)
            include_directories("${PY_NUMPY}/core/include")
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

if(NOT MSVC)
    target_link_flags(ompl)
    set(PKG_NAME "ompl")
    set(PKG_DESC "The Open Motion Planning Library")
    set(PKG_EXTERNAL_DEPS "eigen3 ${ompl_PKG_DEPS}")
    if(OMPL_EXTENSION_ODE)
        set(PKG_EXTERNAL_DEPS "${PKG_EXTERNAL_DEPS}")
    endif()
    set(PKG_OMPL_LIBS "-lompl ${ompl_LINK_FLAGS}")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/ompl.pc.in"
                   "${CMAKE_CURRENT_BINARY_DIR}/ompl.pc" @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ompl.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
        COMPONENT ompl)
endif()

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
            execute_process(COMMAND "${PYTHON_EXEC}" "-m" "webbrowser" "https://ompl.kavrakilab.org/core/register.html"
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
