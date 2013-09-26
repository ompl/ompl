set(OMPL_MORSE_PYTHON_INSTALL_DIR "${OMPL_PYTHON_INSTALL_DIR}/ompl/morse")

configure_file(ompl_addon.py.in ompl_addon.py @ONLY)
configure_file(communicator.py.in communicator.py @ONLY)

install(FILES builder.py ../../build/scripts/morse/communicator.py environment.py
    DESTINATION "${OMPL_MORSE_PYTHON_INSTALL_DIR}")
install(PROGRAMS planner.py player.py
    DESTINATION "${OMPL_MORSE_PYTHON_INSTALL_DIR}")

# TODO need to have FindBlender module
#install(FILES ../../build/scripts/morse/ompl_addon.py DESTINATION "${BLENDER_PLUGIN_DIR}")
