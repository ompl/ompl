install(PROGRAMS ompl_benchmark_statistics.py
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT ompl
    RENAME ompl_benchmark_statistics${OMPL_INSTALL_SUFFIX}.py)
