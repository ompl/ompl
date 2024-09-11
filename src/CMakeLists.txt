add_subdirectory(ompl)
target_include_directories(ompl
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

get_target_property(_include_dirs ompl INCLUDE_DIRECTORIES)
set(OMPL_INCLUDE_DIRS _include_dirs)
get_target_property(_include_dirs ompl INTERFACE_INCLUDE_DIRECTORIES)
list(APPEND OMPL_INCLUDE_DIRS _include_dirs)

install(DIRECTORY ompl/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ompl"
    COMPONENT ompl
    FILES_MATCHING 
    PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)

