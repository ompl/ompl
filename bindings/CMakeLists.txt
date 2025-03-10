cmake_minimum_required(VERSION 3.15...3.27)
project(ompl_bindings)

find_package(Python 3.8
  REQUIRED COMPONENTS Interpreter Development
  OPTIONAL_COMPONENTS Development.SABIModule)

# Import nanobind through CMake's find_package mechanism
add_subdirectory(nanobind)
add_compile_options(-Wno-unused-parameter)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl)
############################################
# Base
############################################
# file(GLOB_RECURSE BASE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/base/*.cpp")
file(GLOB BASE_ROOT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/base/*.cpp")

file(GLOB_RECURSE BASE_SPACES_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/base/spaces/*.cpp")
file(GLOB_RECURSE BASE_TERMINATION_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/base/terminationconditions/*.cpp")
file(GLOB_RECURSE BASE_SAMPLERS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/base/samplers/*.cpp")
file(GLOB_RECURSE BASE_GOAL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/base/goals/*.cpp")
file(GLOB_RECURSE BASE_OBJECTIVES_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/base/objectives/*.cpp")

list(APPEND BASE_SOURCES ${BASE_ROOT_SOURCES} ${BASE_SPACES_SOURCES} ${BASE_TERMINATION_SOURCES} ${BASE_SAMPLERS_SOURCES} ${BASE_GOAL_SOURCES} ${BASE_OBJECTIVES_SOURCES})
nanobind_add_module(_base
    STABLE_ABI
    ${BASE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/base/python.cc
)
set_target_properties(_base PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)
target_link_libraries(_base PRIVATE ompl::ompl)
############################################
# Geometric
############################################
file(GLOB_RECURSE GEOMETRIC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/geometric/*.cpp")
nanobind_add_module(_geometric
    STABLE_ABI
    ${GEOMETRIC_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/geometric/python.cc
)
set_target_properties(_geometric PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)
target_link_libraries(_geometric PRIVATE ompl::ompl)
############################################
# Util
############################################
# Add the module
file(GLOB_RECURSE UTIL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/util/**.cpp")
# MESSAGE(STATUS "UTIL_SOURCES: ${UTIL_SOURCES}")
nanobind_add_module(_util
    STABLE_ABI
    ${UTIL_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/util/python.cc
)
set_target_properties(_util PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)
target_link_libraries(_util PRIVATE ompl::ompl)
############################################
# Control
############################################
file(GLOB_RECURSE CONTROL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/control/*.cpp")
nanobind_add_module(_control
    STABLE_ABI
    ${CONTROL_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/control/python.cc
)
set_target_properties(_control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)
target_link_libraries(_control PRIVATE ompl::ompl)
############################################

set(INIT_PY_CONTENT
"from . import _base as base\n"
"from . import _control as control\n"
"from . import _geometric as geometric\n"
"from . import _util as util\n"
"\n"
)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ompl/__init__.py ${INIT_PY_CONTENT})

############################################
# Generate Type Stubs for Python
############################################

nanobind_add_stub(
    base_stub
    MODULE _base
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ompl/base.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_base>
    DEPENDS _base
)

nanobind_add_stub(
    control_stub
    MODULE _control
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ompl/control.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_control>
    DEPENDS _control
)

nanobind_add_stub(
    geometric_stub
    MODULE _geometric
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ompl/geometric.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_geometric>
    DEPENDS _geometric
)

nanobind_add_stub(
    util_stub
    MODULE _util
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ompl/util.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_util>
    DEPENDS _util
)

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
