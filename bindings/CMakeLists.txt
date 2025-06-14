# ---------------------------------------------------------------------
# Create a single python module named _ompl
# ---------------------------------------------------------------------
file(GLOB_RECURSE PY_OMPL_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} base/*.cpp control/*.cpp geometric/*.cpp util/*.cpp python.cpp)
nanobind_add_module(_ompl
    STABLE_ABI
    ${PY_OMPL_SOURCES}
)

# If a non-standard CMAKE_LIBRARY_OUTPUT_DIRECTORY is defined (for example, by
# the wheel build system), place the shared library artifact underneath it as
# `ompl/_ompl.so`.
#
# Otherwise, do the same but relative to the default library output path.
if (DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set_target_properties(_ompl PROPERTIES LIBRARY_OUTPUT_DIRECTORY
        "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ompl")
else()
    set_target_properties(_ompl PROPERTIES LIBRARY_OUTPUT_DIRECTORY "ompl")
endif()

target_link_libraries(_ompl PRIVATE ompl::ompl)
target_compile_options(_ompl PRIVATE -Wno-unused-parameter)

file(COPY ompl DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

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

install(TARGETS _ompl DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/ompl)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ompl DESTINATION ${OMPL_PYTHON_INSTALL_DIR})
