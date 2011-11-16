option(OMPL_BUILD_PYBINDINGS "Build OMPL Python bindings" ON)
option(OMPL_BUILD_PYTESTS "Build OMPL Python tests" ON)

if (OMPL_BUILD_PYBINDINGS)

    include(PythonBindingsUtils)

    set(OMPL_MODULES base control geometric tools util)

    if(PY_OMPL_GENERATE)
        # need to run cmake to regenerate ompl/bindings_generator.py if
        # bindings_generator.py.in is changed.
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/bindings_generator.py.in"
            "${CMAKE_CURRENT_SOURCE_DIR}/ompl/bindings_generator.py" @ONLY)

        foreach(module ${OMPL_MODULES})
            create_module_generation_targets(${module} ${CMAKE_CURRENT_SOURCE_DIR})
        endforeach(module)

        add_dependencies(update_base_bindings update_util_bindings)
        add_dependencies(update_geometric_bindings update_base_bindings)
        add_dependencies(update_control_bindings update_geometric_bindings)
        add_dependencies(update_control_bindings update_base_bindings)
        #add_dependencies(update_tools_bindings update_control_bindings)

    endif(PY_OMPL_GENERATE)

    if(PY_OMPL_COMPILE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bindings")
        foreach(module ${OMPL_MODULES})
            create_module_target(${module} ${CMAKE_CURRENT_SOURCE_DIR})
        endforeach(module)

        install(DIRECTORY ompl DESTINATION "${OMPL_PYTHON_INSTALL_DIR}"
            PATTERN .svn EXCLUDE PATTERN "*.pyc" EXCLUDE PATTERN "_*.so" EXCLUDE)
    endif()

endif(OMPL_BUILD_PYBINDINGS)
