add_subdirectory(ompl)
install(DIRECTORY ompl/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ompl${OMPL_INSTALL_SUFFIX}"
    COMPONENT ompl
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)
