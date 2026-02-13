# ---------------------------------------------------------------------
# Create a single python module named _ompl
# ---------------------------------------------------------------------
file(GLOB_RECURSE PY_OMPL_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} base/*.cpp control/*.cpp geometric/*.cpp util/*.cpp tools/*.cpp python.cpp)
nanobind_add_module(_ompl
    STABLE_ABI
    ${PY_OMPL_SOURCES}
)

if (DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set_target_properties(_ompl PROPERTIES LIBRARY_OUTPUT_DIRECTORY
        "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ompl")
else()
    set_target_properties(_ompl PROPERTIES LIBRARY_OUTPUT_DIRECTORY "ompl")
endif()

target_link_libraries(_ompl PRIVATE ompl::ompl)
target_compile_options(_ompl PRIVATE -Wno-unused-parameter)

# Set RPATH so the module can find libompl.so
if(UNIX)
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
install(TARGETS _ompl LIBRARY DESTINATION ompl)
# install ompl shared lib
install(TARGETS ompl LIBRARY DESTINATION ompl)
# install python stubs
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl/_ompl
        DESTINATION ompl
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
        DESTINATION ompl)
