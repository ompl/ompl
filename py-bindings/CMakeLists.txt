option(OMPL_BUILD_PYBINDINGS "Build OMPL Python bindings" ON)
option(OMPL_BUILD_PYTESTS "Build OMPL Python tests" ON)

if (OMPL_BUILD_PYBINDINGS)

    include(PythonBindingsUtils)

    if(WIN32)
        # omit StateStorage class bindings on Windows;
        # Boost.Serialization does nasty things that break the bindings.
        set(PYOMPL_EXTRA_CFLAGS "${PYOMPL_EXTRA_CFLAGS} -DOMPL_BASE_STATE_STORAGE_ -DOMPL_BASE_PLANNER_DATA_STORAGE_ -DOMPL_CONTROL_PLANNER_DATA_STORAGE_")
        add_definitions(-DOMPL_BASE_STATE_STORAGE_ -DOMPL_BASE_PLANNER_DATA_STORAGE_ -DOMPL_CONTROL_PLANNER_DATA_STORAGE_)
    endif(WIN32)

    set(OMPL_MODULES base control geometric tools util)
    if(OMPL_EXTENSION_MORSE)
        list(APPEND OMPL_MODULES morse)
    endif()

    if(PY_OMPL_GENERATE)
        # need to run cmake to regenerate ompl/bindings_generator.py if
        # bindings_generator.py.in is changed.
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/bindings_generator.py.in"
            "${CMAKE_CURRENT_SOURCE_DIR}/ompl/bindings_generator.py" @ONLY)

        foreach(module ${OMPL_MODULES})
            create_module_generation_targets(${module} "${CMAKE_CURRENT_SOURCE_DIR}")
        endforeach(module)

        add_dependencies(update_base_bindings update_util_bindings)
        add_dependencies(update_geometric_bindings update_base_bindings)
        add_dependencies(update_control_bindings update_geometric_bindings)
        add_dependencies(update_control_bindings update_base_bindings)
        add_dependencies(update_tools_bindings update_control_bindings)
        if(OMPL_EXTENSION_MORSE)
            add_dependencies(update_morse_bindings update_base_bindings)
            add_dependencies(update_morse_bindings update_control_bindings)
        endif()
    endif(PY_OMPL_GENERATE)

    if(PY_OMPL_COMPILE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bindings")
        foreach(module ${OMPL_MODULES})
            create_module_target(${module} "${CMAKE_CURRENT_SOURCE_DIR}")
        endforeach(module)

        install(DIRECTORY ompl DESTINATION "${OMPL_PYTHON_INSTALL_DIR}"
            COMPONENT python
            PATTERN "*.pyc" EXCLUDE
            PATTERN "__pycache__" EXCLUDE
            PATTERN "_*.so" EXCLUDE
            PATTERN "app" EXCLUDE
            PATTERN "morse" EXCLUDE)
        # put app submodule in omplapp component
        if(OMPLAPP_VERSION)
            install(DIRECTORY ompl/app
                DESTINATION "${OMPL_PYTHON_INSTALL_DIR}/ompl"
                COMPONENT omplapp
                PATTERN "*.pyc" EXCLUDE
                PATTERN "__pycache__" EXCLUDE
                PATTERN "_*.so" EXCLUDE)
        endif()
        # put morse submodule in morse component
        if(OMPL_EXTENSION_MORSE)
            install(DIRECTORY ompl/morse
                DESTINATION "${OMPL_PYTHON_INSTALL_DIR}/ompl"
                COMPONENT morse
                PATTERN "*.pyc" EXCLUDE
                PATTERN "__pycache__" EXCLUDE
                PATTERN "_*.so" EXCLUDE
                PATTERN "addons" EXCLUDE)
            # blender add-on
            install(DIRECTORY ompl/morse/addons
                DESTINATION "share/ompl${OMPL_INSTALL_SUFFIX}"
                COMPONENT morse
                PATTERN "*.pyc" EXCLUDE)
        endif()
    endif()

endif(OMPL_BUILD_PYBINDINGS)

add_custom_target(clean_bindings
    "${CMAKE_COMMAND}" -E remove_directory "${CMAKE_CURRENT_SOURCE_DIR}/bindings"
    COMMAND "${CMAKE_COMMAND}" -E remove -f pyplusplus_{base,control,geometric,tools,util}.{cache,log}
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}")
