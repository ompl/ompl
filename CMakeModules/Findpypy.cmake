include(FindPackageHandleStandardArgs)

# PyPy is more than an order of magnitude faster in generating the Python
# bindings than CPython, so use it if available.
find_program(PYPY NAMES pypy${PYTHON_VERSION_MAJOR} pypy)
if(PYPY)
    option(OMPL_USE_PYPY "Use PyPy for generating Python bindings" ON)
else()
    option(OMPL_USE_PYPY "Use PyPy for generating Python bindings" OFF)
endif()
add_feature_info(OMPL_USE_PYPY "${OMPL_USE_PYPY}" "Use PyPy for generating Python bindings.")

if(OMPL_USE_PYPY)
    set(PYTHON_BINDING_EXEC "${PYPY}")
else()
    set(PYTHON_BINDING_EXEC "${PYTHON_EXEC}")
endif()
find_package_handle_standard_args(pypy DEFAULT_MSG PYPY)
