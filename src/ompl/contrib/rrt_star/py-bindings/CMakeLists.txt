if (OMPL_BUILD_PYBINDINGS)
    include(PythonBindingsUtils)

    # Put the RRT* bindings in the same directory as the python bindings for
    # the rest of OMPL.
    set(OMPL_RRTSTAR_BINDINGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../py-bindings")

    # Copy the rrtstar module's __init__.py
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/rrtstar/__init__.py"
        "${OMPL_RRTSTAR_BINDINGS_DIR}/ompl/rrtstar/__init__.py")

    # Create the target "update_rrtstar_bindings", which will (re)generate
    # the binding code
    if(PY_OMPL_GENERATE)
        create_module_generation_targets(rrtstar "${CMAKE_CURRENT_SOURCE_DIR}")
        add_dependencies(update_rrtstar_bindings update_geometric_bindings)
    endif()

    # Create the target "py_ompl_rrtstar", which will compile the binding
    # code into a binary python module
    if(PY_OMPL_COMPILE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bindings")
        create_module_target(rrtstar ${CMAKE_CURRENT_SOURCE_DIR}
            "${OMPL_RRTSTAR_BINDINGS_DIR}/ompl")
    endif()
endif()

# Not strictly necessary, but can be convenient if the API of the RRT*
# planners changes
add_custom_target(clean_rrtstar_bindings
    ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/bindings
    COMMAND ${CMAKE_COMMAND} -E remove -f pyplusplus_rrtstar.{cache,log}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
# clean RRT* bindings whenever the target clean_bindings is built
add_dependencies(clean_bindings clean_rrtstar_bindings)
