# ---------------------------------------------------------------------
# Create a single python module named _ompl
# ---------------------------------------------------------------------
file(GLOB_RECURSE PY_OMPL_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
nanobind_add_module(_ompl
    STABLE_ABI
    ${PY_OMPL_SOURCES}
)

set_target_properties(_ompl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl
)
target_link_libraries(_ompl PRIVATE ompl::ompl)
target_compile_options(_ompl PRIVATE -Wno-unused-parameter)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl)
file(COPY_FILE __init__.py ${CMAKE_CURRENT_BINARY_DIR}/ompl/__init__.py ONLY_IF_DIFFERENT)

# ---------------------------------------------------------------------
# Generate type stubs for the single module
# ---------------------------------------------------------------------
nanobind_add_stub(
    ompl_stub
    MODULE _ompl
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ompl/ompl.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_ompl>
    DEPENDS _ompl
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl DESTINATION ${OMPL_PYTHON_INSTALL_DIR})