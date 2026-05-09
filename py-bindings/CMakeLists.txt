# ---------------------------------------------------------------------
# Create a single python module named _ompl
# ---------------------------------------------------------------------
file(GLOB_RECURSE PY_OMPL_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} base/*.cpp control/*.cpp geometric/*.cpp util/*.cpp tools/*.cpp python.cpp)
nanobind_add_module(_ompl
    STABLE_ABI
    ${PY_OMPL_SOURCES}
)

# Default to Python_SITEARCH only for system-wide installs; for custom
# prefixes use a path relative to CMAKE_INSTALL_PREFIX so install() works
# without root and without writing outside the requested prefix.
if(NOT OMPL_PYTHON_INSTALL_PREFIX AND NOT SKBUILD)
    if(DEFINED ENV{DEB_BUILD_OPTIONS}
       OR CMAKE_INSTALL_PREFIX MATCHES "^/usr/?$"
       OR CMAKE_INSTALL_PREFIX MATCHES "^/usr/local/?$")
        set(_ompl_default_py_install "${Python_SITEARCH}")
    else()
        set(_ompl_default_py_install
            "${CMAKE_INSTALL_LIBDIR}/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages")
    endif()
    set(OMPL_PYTHON_INSTALL_PREFIX "${_ompl_default_py_install}" CACHE PATH
        "Path to directory where OMPL python module will be installed")
    unset(_ompl_default_py_install)
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
