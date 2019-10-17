# The python version needs to match the one used to build Boost.Python.
# You can optionally specify the desired version like so:
#   find_package(Python 2.6)
find_package(Python QUIET)
find_boost_python()

find_python_module(pyplusplus 1.6.0)
find_python_module(pygccxml 1.7.2)
find_package(castxml)

if(PYTHON_FOUND AND Boost_PYTHON_LIBRARY)
    include_directories(SYSTEM ${PYTHON_INCLUDE_DIRS})
    # make sure target is defined only once
    if(NOT TARGET py_ompl)
        # top-level target for compiling python modules
        add_custom_target(py_ompl)
    endif()
    set(PY_OMPL_COMPILE ON CACHE BOOL
        "Whether the OMPL Python modules can be built")
    mark_as_advanced(PY_OMPL_COMPILE)
    set(OMPL_PYTHON_INSTALL_DIR "${PYTHON_SITE_MODULES}" CACHE STRING
        "Path to directory where OMPL python modules will be installed")
endif()

if(PYTHON_FOUND AND Boost_PYTHON_LIBRARY AND PY_PYPLUSPLUS
    AND PY_PYGCCXML AND CASTXML)
    # make sure targets are defined only once
    if(NOT TARGET generate_headers)
        # top-level target for updating all-in-one header file for each module
        add_custom_target(generate_headers)
        # top-level target for regenerating code for all python modules
        add_custom_target(update_bindings)
    endif()
    set(PY_OMPL_GENERATE ON CACHE BOOL
        "Whether the C++ code for the OMPL Python module can be generated")
    mark_as_advanced(PY_OMPL_GENERATE)

    get_filename_component(PY_PYPLUSPLUS_DIR "${PY_PYPLUSPLUS}" DIRECTORY)
    get_filename_component(PY_PYGCCXML_DIR "${PY_PYGCCXML}" DIRECTORY)
    set(PYTHONPATH "${PY_PYPLUSPLUS_DIR}:${PY_PYGCCXML_DIR}:$ENV{PYTHONPATH}" CACHE STRING "Python path for pyplusplus and pygccxml")
endif()

function(create_module_header_file_target module)
    # create list of absolute paths to header files, which we
    # will add as a list of dependencies for the target
    file(READ "headers_${module}.txt" headers_string)
    separate_arguments(rel_headers UNIX_COMMAND "${headers_string}")
    set(headers "")
    foreach(header ${rel_headers})
        get_filename_component(header "../${header}" ABSOLUTE)
        list(APPEND headers "${header}")
    endforeach(header)
    # target for all-in-one header for module
    add_custom_target(${module}.h
        COMMAND ${CMAKE_COMMAND} -D dir="${CMAKE_CURRENT_SOURCE_DIR}/.." -D module=${module} -D exclude=${ARGV1}
        -P "${OMPL_CMAKE_UTIL_DIR}/generate_header.cmake"
        DEPENDS ${headers}
        COMMENT "Preparing C++ header file for Python binding generation for module ${module}")
    add_dependencies(generate_headers ${module}.h)
endfunction(create_module_header_file_target)

function(create_module_code_generation_target module)
    # target for regenerating code. Cmake is run so that the list of
    # sources for the py_ompl_${module} target (see below) is updated.
    add_custom_target(update_${module}_bindings
        COMMAND env PYTHONPATH=\"${PYTHONPATH}\" ${PYTHON_BINDING_EXEC}
        "${CMAKE_CURRENT_SOURCE_DIR}/generate_bindings.py" "${module}"
        "2>&1" | tee "${CMAKE_BINARY_DIR}/pyplusplus_${module}.log"
        COMMAND ${CMAKE_COMMAND} ${CMAKE_BINARY_DIR}
        COMMENT "Creating C++ code for Python module ${module} (see pyplusplus_${module}.log)")
    add_dependencies(update_${module}_bindings ${module}.h)
    add_dependencies(update_bindings update_${module}_bindings)
endfunction(create_module_code_generation_target)

function(create_module_generation_targets module)
    create_module_header_file_target("${module}" "${ARGV1}")
    create_module_code_generation_target("${module}")
endfunction(create_module_generation_targets)

function(create_module_target module)
    # target for each python module
    aux_source_directory("${CMAKE_CURRENT_BINARY_DIR}/bindings/${module}" PY${module}BINDINGS)
    list(LENGTH PY${module}BINDINGS NUM_SOURCE_FILES)
    if(NUM_SOURCE_FILES GREATER 0)
        if(ARGC GREATER 1)
            set(_dest_dir "${ARGV1}")
            if(ARGC GREATER 2)
                set(_extra_libs "${ARGV2}")
            endif()
        else()
            set(_dest_dir "${CMAKE_CURRENT_SOURCE_DIR}/ompl")
        endif()
        if(WIN32)
            # If this is built as a 'module', the compiler complains upon link,
            # complaining that the symbol WinMain is undefined.
            # Python on windows will NOT read this file unless the extension is .pyd
            add_library(py_ompl_${module} SHARED ${PY${module}BINDINGS})
            set_target_properties(py_ompl_${module} PROPERTIES OUTPUT_NAME ${module} PREFIX _ SUFFIX .pyd)
        else(WIN32)
            add_library(py_ompl_${module} MODULE ${PY${module}BINDINGS})
            set_target_properties(py_ompl_${module} PROPERTIES OUTPUT_NAME _${module})
            if(CMAKE_CXX_COMPILER_ID MATCHES "^(Apple)?Clang$")
                target_compile_options(py_ompl_${module} PRIVATE -Wno-unused-local-typedef)
            endif()
            if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
                target_compile_options(py_ompl_${module} PRIVATE -Wno-unused-local-typedefs)
            endif()
        endif(WIN32)

        target_link_libraries(py_ompl_${module}
            ompl
            ${_extra_libs}
            ${Boost_PYTHON_LIBRARY}
            ${PYTHON_LIBRARIES})
        add_dependencies(py_ompl py_ompl_${module})
        if(WIN32)
            add_custom_command(TARGET py_ompl_${module} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:py_ompl_${module}>"
                "${_dest_dir}/${module}/_${module}.pyd"
                WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
                COMMENT "Copying python module ${module} into place")
        else(WIN32)
            add_custom_command(TARGET py_ompl_${module} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:py_ompl_${module}>"
                "${_dest_dir}/${module}/_${module}${CMAKE_SHARED_MODULE_SUFFIX}"
                WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
                COMMENT "Copying python module ${module} into place")
        endif(WIN32)
        # put omplapp and MORSE bindings in separate components
        if(${module} STREQUAL "app")
            set(_component "omplapp")
        elseif(${module} STREQUAL "morse")
            set(_component "morse")
        else()
            set(_component "python")
        endif()
        install(TARGETS py_ompl_${module}
            DESTINATION "${OMPL_PYTHON_INSTALL_DIR}/ompl/${module}/"
            COMPONENT ${_component})
        include_directories("${CMAKE_CURRENT_BINARY_DIR}/bindings/${module}" "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    else(NUM_SOURCE_FILES GREATER 0)
        if(PY_OMPL_GENERATE)
            message(STATUS "Code for module ${module} not found; type \"make update_bindings\"")
        else()
            message(STATUS "Code for module ${module} not found")
        endif()
    endif(NUM_SOURCE_FILES GREATER 0)
endfunction(create_module_target)
