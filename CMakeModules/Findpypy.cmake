include(FindPackageHandleStandardArgs)

# PyPy is more than an order of magnitude faster in generating the Python
# bindings than CPython, so use it if available.
find_program(PYPY NAMES pypy${PYTHON_VERSION_MAJOR} pypy)

if(PYPY)
  execute_process(
    COMMAND ${PYPY} "-c" "import platform; print(platform.python_version())"
    OUTPUT_VARIABLE PYPY_PYTHON_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(${PYPY_PYTHON_VERSION} VERSION_GREATER_EQUAL ${PYTHON_VERSION_MAJOR})
    set(PYTHON_BINDING_EXEC "${PYPY}")
  else()
    set(PYTHON_BINDING_EXEC "${PYTHON_EXEC}")
    message(STATUS "Could NOT use pypy ${PYPY_PYTHON_VERSION} - incompatable with Python version ${PYTHON_VERSION_FULL}")
  endif()
else()
  set(PYTHON_BINDING_EXEC "${PYTHON_EXEC}")
endif()

find_package_handle_standard_args(pypy DEFAULT_MSG PYPY)
