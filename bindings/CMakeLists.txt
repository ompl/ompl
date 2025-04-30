# cmake_minimum_required(VERSION 3.15...3.27)
# project(ompl_bindings)

# # ---------------------------------------------------------------------
# # 1) Find Python and add nanobind
# # ---------------------------------------------------------------------
# find_package(Python 3.8
#   REQUIRED COMPONENTS Interpreter Development
#   OPTIONAL_COMPONENTS Development.SABIModule
# )
# add_subdirectory(nanobind)
add_compile_options(-Wno-unused-parameter)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl)

# ---------------------------------------------------------------------
# 2) Gather All Source Files
# ---------------------------------------------------------------------
#
# We'll collect all source files from base, geometric, control, util, etc.
# and combine them into one set of sources for a single extension module.

# A) Collect "base" sources
file(GLOB BASE_ROOT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/base/*.cpp"
)
file(GLOB_RECURSE BASE_SPACES_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/base/spaces/*.cpp"
)
file(GLOB_RECURSE BASE_TERMINATION_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/base/terminationconditions/*.cpp"
)
file(GLOB_RECURSE BASE_SAMPLERS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/base/samplers/*.cpp"
)
file(GLOB_RECURSE BASE_GOAL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/base/goals/*.cpp"
)
file(GLOB_RECURSE BASE_OBJECTIVES_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/base/objectives/*.cpp"
)

list(APPEND ALL_SOURCES
    ${BASE_ROOT_SOURCES}
    ${BASE_SPACES_SOURCES}
    ${BASE_TERMINATION_SOURCES}
    ${BASE_SAMPLERS_SOURCES}
    ${BASE_GOAL_SOURCES}
    ${BASE_OBJECTIVES_SOURCES}
)

# B) Collect "geometric" sources
file(GLOB BASE_GEOMETRIC_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/geometric/*.cpp"
)
file(GLOB_RECURSE GEOMETRIC_PLANNERS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/geometric/planners/*.cpp"
)

list(APPEND ALL_SOURCES
    ${BASE_GEOMETRIC_SOURCES}
    ${GEOMETRIC_PLANNERS_SOURCES}
)

# C) Collect "control" sources
file(GLOB CONTROL_ROOT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/control/*.cpp"
)
file(GLOB_RECURSE CONTROL_PLANNERS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/control/planners/*.cpp"
)
file(GLOB_RECURSE CONTROL_SPACES_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/control/spaces/*.cpp"
)
list(APPEND ALL_SOURCES
    ${CONTROL_ROOT_SOURCES}
    ${CONTROL_PLANNERS_SOURCES}
    ${CONTROL_SPACES_SOURCES}
)

# D) Collect "util" sources
file(GLOB_RECURSE UTIL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp"
)
list(APPEND ALL_SOURCES
    ${UTIL_SOURCES}
)

# ---------------------------------------------------------------------
# 3) Create a single nanobind module named _ompl
# ---------------------------------------------------------------------
nanobind_add_module(_ompl
    STABLE_ABI
    ${ALL_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/python.cc
)

set_target_properties(_ompl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)
target_link_libraries(_ompl PRIVATE ompl::ompl)

# ---------------------------------------------------------------------
# 4) Create __init__.py
# ---------------------------------------------------------------------
set(INIT_PY_CONTENT
"from ._ompl import base
from ._ompl import geometric
from ._ompl import control
from ._ompl import util"
)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ompl/__init__.py ${INIT_PY_CONTENT})

# ---------------------------------------------------------------------
# 5) Generate type stubs for the single module
# ---------------------------------------------------------------------
nanobind_add_stub(
    ompl_stub
    INSTALL_TIME
    MODULE _ompl
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ompl/ompl.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_ompl>
    DEPENDS _ompl
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
