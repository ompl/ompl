install(PROGRAMS ompl_benchmark_statistics.py
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT ompl
    RENAME ompl_benchmark_statistics${OMPL_INSTALL_SUFFIX}.py)

find_program(R_EXEC R)
if (R_EXEC)
    install(DIRECTORY plannerarena
        DESTINATION "share/ompl${OMPL_INSTALL_SUFFIX}"
        COMPONENT ompl)
    install(PROGRAMS plannerarena/plannerarena
        DESTINATION bin
        RENAME plannerarena${OMPL_INSTALL_SUFFIX})
endif()
