configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in" "${CMAKE_CURRENT_BINARY_DIR}/config.h")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ompl"
    COMPONENT ompl)

file(GLOB_RECURSE OMPL_SOURCE_CODE datastructures/*.cpp util/*.cpp base/*.cpp
  geometric/*.cpp control/*.cpp multilevel/*.cpp tools/*.cpp)
file(GLOB_RECURSE OMPL_HEADERS datastructures/*.h util/*.h base/*.h
  geometric/*.h control/*.h multilevel/*.h tools/*.h)

#############################################
# Add enabled extensions to the source code #
#############################################

## replace XXX by the extension
#if (OMPL_EXTENSION_XXX)
#    file(GLOB_RECURSE OMPL_XXX_EXTENSION_SOURCE_CODE extensions/XXX/*.cpp)
#    list(APPEND OMPL_SOURCE_CODE ${OMPL_XXX_EXTENSION_SOURCE_CODE})
#
##    If additional libraries need to be linked, they are added to package.xml for ROS,
##    or, they are added to the list of targets linked to ompl. Finally, add them to
##    the config script as dependencies
#
#endif()

if (OMPL_EXTENSION_TRIANGLE)
    file(GLOB_RECURSE OMPL_TRIANGLE_EXTENSION_SOURCE_CODE extensions/triangle/*.cpp)
    file(GLOB_RECURSE OMPL_TRIANGLE_EXTENSION_HEADERS extensions/triangle/*.h)
    list(APPEND OMPL_SOURCE_CODE ${OMPL_TRIANGLE_EXTENSION_SOURCE_CODE})
    list(APPEND OMPL_HEADERS ${OMPL_TRIANGLE_EXTENSION_HEADERS})
endif()

source_group("OMPL Source" FILES "${OMPL_SOURCE_CODE}")
source_group("OMPL Headers" FILES "${OMPL_HEADERS}")

# build the library
if(MSVC)
    add_library(ompl STATIC ${OMPL_SOURCE_CODE})
else()
    add_library(ompl SHARED ${OMPL_SOURCE_CODE})
endif()
add_library(ompl::ompl ALIAS ompl)
target_link_libraries(ompl
    PUBLIC
        Boost::filesystem
        Boost::serialization
        Boost::system
        Eigen3::Eigen
        "$<$<BOOL:${Threads_FOUND}>:Threads::Threads>"
        "$<$<BOOL:${OMPL_HAVE_FLANN}>:flann::flann>"
        "$<$<BOOL:${OMPL_EXTENSION_TRIANGLE}>:Triangle::Triangle>"
    PRIVATE
        "$<$<BOOL:${OMPL_HAVE_SPOT}>:Spot::Spot>")

if (MSVC)
    set_target_properties(ompl PROPERTIES VERSION "${PROJECT_VERSION}" STATIC_LIBRARY_FLAGS "psapi.lib ws2_32.lib")
else (MSVC)
    if (MINGW)
        target_link_libraries(ompl psapi ws2_32)
        set_target_properties(ompl PROPERTIES LINK_FLAGS "-Wl,--export-all-symbols")
    endif (MINGW)
    set_target_properties(ompl PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION "${OMPL_ABI_VERSION}")
endif (MSVC)

if (NOT MSVC)
    add_custom_command(TARGET ompl POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy "$<TARGET_FILE:ompl>"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../py-bindings/ompl/util/libompl${CMAKE_SHARED_LIBRARY_SUFFIX}")
endif (NOT MSVC)
