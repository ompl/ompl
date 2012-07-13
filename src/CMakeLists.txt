add_subdirectory(ompl)
install(DIRECTORY ompl/ DESTINATION "include/ompl${OMPL_INSTALL_SUFFIX}"
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN ".svn" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)
ompl_install_symlink(include ompl)

install(DIRECTORY external/omplext_odeint/ DESTINATION "include/omplext_odeint${OMPL_INSTALL_SUFFIX}")
ompl_install_symlink(include omplext_odeint)

# Add target to install Py++ and its dependencies pygccxml and gccxml
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh" @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.bat.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.bat" @ONLY)
if(WIN32)
add_custom_target(installpyplusplus
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.bat" VERBATIM)
else(WIN32)
add_custom_target(installpyplusplus
    COMMAND "sh" "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh")
endif(WIN32)
