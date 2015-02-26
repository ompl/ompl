install(PROGRAMS ompl_benchmark_statistics.py
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT ompl
    RENAME ompl_benchmark_statistics${OMPL_INSTALL_SUFFIX}.py)

install(DIRECTORY plannerarena
    DESTINATION "share/ompl${OMPL_INSTALL_SUFFIX}"
    COMPONENT ompl)
