add_subdirectory(ompl)
install(DIRECTORY ompl DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN ".svn" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)
    
install(DIRECTORY external/omplext_odeint DESTINATION include)

# Add target to install Py++ and its dependencies pygccxml and gccxml
# (This won't work on Windows, since there is no "patch" command.)
set(PATCH_GCCXCML_DIFF "${CMAKE_CURRENT_SOURCE_DIR}/external/patch-pygccxml.diff")
# This patch is for Windows, and includes the diff above, as well as another Windows specific change.
set(PATCH_GCCXML_SOURCE_READER "${CMAKE_CURRENT_SOURCE_DIR}/external/source_reader.py")
string(REPLACE "/" "\\" PATCH_GCCXML_SOURCE_READER ${PATCH_GCCXML_SOURCE_READER})
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
