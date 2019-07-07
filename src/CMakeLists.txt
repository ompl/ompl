add_subdirectory(ompl)
install(DIRECTORY ompl/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ompl"
    COMPONENT ompl
    FILES_MATCHING 
    PATTERN "*.ipp" 
    PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    PATTERN "*.ipp"
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)
