add_subdirectory(ompl)
install(DIRECTORY ompl DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN ".svn" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)

# Add target to install Py++ and its dependencies pygccxml and gccxml
# (This won't work on Windows, since there is no "patch" command.)
set(PATCH_GCCXCML_DIFF "${CMAKE_CURRENT_SOURCE_DIR}/external/patch-pygccxml.diff")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh" @ONLY)
add_custom_target(installpyplusplus
    COMMAND "sh" "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh")
