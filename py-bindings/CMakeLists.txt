# ---------------------------------------------------------------------
# Create a single python module named _ompl
# ---------------------------------------------------------------------
file(GLOB_RECURSE PY_OMPL_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} base/*.cpp control/*.cpp geometric/*.cpp util/*.cpp tools/*.cpp python.cpp)
nanobind_add_module(_ompl
    STABLE_ABI
    ${PY_OMPL_SOURCES}
)

# Ask Python's sysconfig where 'purelib' lands under CMAKE_INSTALL_PREFIX, then
# strip the prefix to get a relative path. Matches ament_cmake_python's
# algorithm so the result honors distro conventions (dist-packages on Debian,
# site-packages elsewhere) and works for any prefix.
if(NOT OMPL_PYTHON_INSTALL_PREFIX AND NOT SKBUILD)
    set(_ompl_py_install_dir_code
"import os, sysconfig
schemes = sysconfig.get_scheme_names()
kwargs = {'vars': {'base': '${CMAKE_INSTALL_PREFIX}'}}
if 'deb_system' in schemes or 'osx_framework_library' in schemes:
    kwargs['scheme'] = 'posix_prefix'
elif 'rpm_prefix' in schemes:
    kwargs['scheme'] = 'rpm_prefix'
print(os.path.relpath(sysconfig.get_path('purelib', **kwargs),
                      start='${CMAKE_INSTALL_PREFIX}').replace(os.sep, '/'))")
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "${_ompl_py_install_dir_code}"
        OUTPUT_VARIABLE _ompl_default_py_install
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _ompl_py_query_result)
    if(NOT _ompl_py_query_result EQUAL 0)
        message(FATAL_ERROR
            "Failed to query Python sysconfig for default install path "
            "(exit ${_ompl_py_query_result}). Set OMPL_PYTHON_INSTALL_PREFIX manually.")
    endif()
    set(OMPL_PYTHON_INSTALL_PREFIX "${_ompl_default_py_install}" CACHE PATH
        "Path to directory where OMPL python module will be installed")
endif()
message(STATUS "OMPL Python module install location: ${OMPL_PYTHON_INSTALL_PREFIX}")
if(OMPL_PYTHON_INSTALL_PREFIX)
    set(OMPL_PYTHON_INSTALL_DIR "${OMPL_PYTHON_INSTALL_PREFIX}/ompl")
else()
    # For scikit-build / wheel builds, relative path
    set(OMPL_PYTHON_INSTALL_DIR "ompl")
endif()

if (DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set_target_properties(_ompl PROPERTIES LIBRARY_OUTPUT_DIRECTORY
        "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ompl")
else()
    set_target_properties(_ompl PROPERTIES LIBRARY_OUTPUT_DIRECTORY "ompl")
endif()

target_link_libraries(_ompl PRIVATE ompl::ompl)
target_compile_options(_ompl PRIVATE -Wno-unused-parameter)

# Set RPATH so the module can find libompl
if(APPLE)
    set_target_properties(_ompl PROPERTIES
        BUILD_RPATH "@loader_path"
        INSTALL_RPATH "@loader_path"
    )
elseif(UNIX)
    set_target_properties(_ompl PROPERTIES
        BUILD_RPATH "$ORIGIN"
        INSTALL_RPATH "$ORIGIN"
    )
endif()

# ---------------------------------------------------------------------
# Generate type stubs for the single module
# ---------------------------------------------------------------------
nanobind_add_stub(
    ompl_stub
    MODULE _ompl
    PYTHON_PATH $<TARGET_FILE_DIR:_ompl>
    DEPENDS _ompl
    VERBOSE
    MARKER_FILE py.typed
    RECURSIVE
    OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/ompl
    OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/ompl/_ompl/__init__.pyi
        ${CMAKE_CURRENT_BINARY_DIR}/ompl/_ompl/base.pyi
        ${CMAKE_CURRENT_BINARY_DIR}/ompl/_ompl/geometric.pyi
        ${CMAKE_CURRENT_BINARY_DIR}/ompl/_ompl/control.pyi
        ${CMAKE_CURRENT_BINARY_DIR}/ompl/_ompl/util.pyi
)

# Post-process .pyi files to fix imports: replace "import _ompl.X" with "from ompl import _ompl"
# This allows references like "_ompl.base.State" to continue working
add_custom_command(
    TARGET ompl_stub POST_BUILD
    COMMAND ${CMAKE_COMMAND} -D dir="${CMAKE_CURRENT_BINARY_DIR}/.."
        -P "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/fix_pyi_imports.cmake"
    COMMENT "Post-processing .pyi stubs to use 'import ompl as _ompl'"
)

# install module
install(TARGETS _ompl LIBRARY DESTINATION ${OMPL_PYTHON_INSTALL_DIR})
# install ompl shared lib
install(TARGETS ompl DESTINATION ${OMPL_PYTHON_INSTALL_DIR})
# install python stubs
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl/_ompl
        DESTINATION ${OMPL_PYTHON_INSTALL_DIR}
        FILES_MATCHING PATTERN "*.pyi")
# install module hierarchy
install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/ompl/__init__.py
        ${CMAKE_CURRENT_SOURCE_DIR}/ompl/base.py
        ${CMAKE_CURRENT_SOURCE_DIR}/ompl/geometric.py
        ${CMAKE_CURRENT_SOURCE_DIR}/ompl/control.py
        ${CMAKE_CURRENT_SOURCE_DIR}/ompl/util.py
        ${CMAKE_CURRENT_SOURCE_DIR}/ompl/tools.py
        ${CMAKE_CURRENT_SOURCE_DIR}/ompl/_benchmark_statistics.py
        DESTINATION ${OMPL_PYTHON_INSTALL_DIR})
if(SKBUILD)
    install(FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/ompl_benchmark_statistics.py
            DESTINATION ${OMPL_PYTHON_INSTALL_DIR})
endif()
