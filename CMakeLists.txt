option(OMPL_IN_ROS "Build OMPL against ROS" OFF)

set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")
set(OMPL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(OMPL_DEMO_INSTALL_DIR share/ompl/demos
    CACHE STRING "Relative path to directory where demos will be installed")
set(OMPL_DOC_INSTALL_DIR share/ompl/doc
    CACHE STRING "Relative path to directory where documentation will be installed")

include_directories("${OMPL_INCLUDE_DIR}")

if(OMPL_IN_ROS)
    rosbuild_add_boost_directories()
    set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
else(OMPL_IN_ROS)
    cmake_minimum_required(VERSION 2.8)
    project(ompl C CXX)
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
    endif()
    enable_testing()

    # ODE is only needed for Open Dynamics Engine bindings
    find_package(OpenDE QUIET)
    set(OMPL_EXTENSION_OPENDE ${OPENDE_FOUND})
    if (OPENDE_FOUND)
        add_definitions(${OPENDE_DEFINITIONS})
        include_directories(${OPENDE_INCLUDE_DIR})
    endif()
endif(OMPL_IN_ROS)

include(OMPLVersion)
include(OMPLUtils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_subdirectory(src)
add_subdirectory(py-bindings)
add_subdirectory(tests)
add_subdirectory(demos)
add_subdirectory(doc)

if(NOT OMPL_IN_ROS)
    include(CPackSettings)
endif()
