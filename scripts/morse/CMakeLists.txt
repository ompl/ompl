set(OMPL_MORSE_PYTHON_INSTALL_DIR "${OMPL_PYTHON_INSTALL_DIR}/ompl/morse")
install(FILES builder.py communicator.py environment.py planner.py player.py
    DESTINATION "${OMPL_MORSE_PYTHON_INSTALL_DIR}")

configure_file(ompl_addon.py.in ompl_addon.py @ONLY)

# TODO need to have FindBlender module
#install(FILES ompl_addon.py DESTINATION "${BLENDER_PLUGIN_DIR}")
