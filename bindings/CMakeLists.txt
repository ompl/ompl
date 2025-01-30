cmake_minimum_required(VERSION 3.15...3.27)
project(ompl_bindings)

find_package(Python 3.8
  REQUIRED COMPONENTS Interpreter Development
  OPTIONAL_COMPONENTS Development.SABIModule)

# Import nanobind through CMake's find_package mechanism
add_subdirectory(nanobind)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl)
############################################
# Base
############################################
file(GLOB BASE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/base/*.cpp")
nanobind_add_module(base
    STABLE_ABI
    ${BASE_SOURCES}
)
set_target_properties(base PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)
target_link_libraries(base PRIVATE ompl::ompl)
############################################
# Geometric
############################################
file(GLOB GEOMETRIC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/geometric/*.cpp")
nanobind_add_module(geometric
    STABLE_ABI
    ${GEOMETRIC_SOURCES}
)
set_target_properties(geometric PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)
target_link_libraries(geometric PRIVATE ompl::ompl)
############################################
# Util
############################################
# Add the module
nanobind_add_module(util
    STABLE_ABI
    util/geometric_equations_bindings.cpp
    util/random_numbers_bindings.cpp
    util/ppm_bindings.cpp
    util/prolate_hyperspheroid_bindings.cpp
    util/all_bindings.cpp
)

set_target_properties(util PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)

target_link_libraries(util PRIVATE ompl::ompl)
############################################

# Warn if the user invokes CMake directly
if (NOT SKBUILD)
  message(WARNING "\
  This CMake file is meant to be executed using 'scikit-build-core'.
  Running it directly will almost certainly not produce the desired
  result. If you are a user trying to install this package, use the
  command below, which will install all necessary build dependencies,
  compile the package in an isolated environment, and then install it.
  =====================================================================
   $ pip install .
  =====================================================================
  If you are a software developer, and this is your own package, then
  it is usually much more efficient to install the build dependencies
  in your environment once and use the following command that avoids
  a costly creation of a new virtual environment at every compilation:
  =====================================================================
   $ pip install nanobind scikit-build-core[pyproject]
   $ pip install --no-build-isolation -ve .
  =====================================================================
  You may optionally add -Ceditable.rebuild=true to auto-rebuild when
  the package is imported. Otherwise, you need to rerun the above
  after editing C++ files.")
endif()

# install(TARGETS _util_ext 
#     LIBRARY 
#     DESTINATION ompl/util)
