if(NOT SKBUILD)
    install(PROGRAMS ompl_benchmark_statistics.py
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT ompl)
endif()
