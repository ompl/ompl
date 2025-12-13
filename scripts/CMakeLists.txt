find_program(R_EXEC R)
if (R_EXEC)
    install(DIRECTORY plannerarena
        DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl"
        COMPONENT ompl)
    install(PROGRAMS plannerarena/plannerarena
        DESTINATION bin
        RENAME plannerarena)
endif()
